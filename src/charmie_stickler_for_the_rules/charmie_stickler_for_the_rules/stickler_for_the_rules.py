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
from charmie_interfaces.msg import RobotSpeech, SpeechType, TarNavSDNL, Yolov8Pose, NeckPosition, SearchForPerson, ListOfPoints, TrackPerson, DetectedPerson, ListOfStrings

import time

import math

class SFTRNode(Node):

    def __init__(self):
        super().__init__("SFTRNode")
        self.get_logger().info("Initiliased SFTR Node")
        
        #RGB
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)
        
        # Neck Topics
        self.neck_position_publisher = self.create_publisher(NeckPosition, "neck_to_pos", 10)
        self.neck_follow_person_publisher = self.create_publisher(TrackPerson, "neck_follow_person", 10)
        self.neck_to_coords_publisher = self.create_publisher(Pose2D, "neck_to_coords", 10)
        #self.neck_get_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos", self.get_neck_position_callback, 10)

        # Low Level Topics
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
        self.flag_start_button_publisher = self.create_publisher(Bool, "flag_start_button", 10)
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
    
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
        self.yolov8_pose_subscriber = self.create_subscription(Yolov8Pose, "person_pose_filtered", self.yolov8_pose_callback, 10)
        self.only_detect_person_right_in_front_publisher = self.create_publisher(Bool, "only_det_per_right_in_front", 10)
        
        #Comms with Main
        self.get_person_publisher = self.create_publisher(Bool, "get_person", 10)
        #self.person_info_subscriber = self.create_subscription(Pose2D, "person_info", self.person_info_callback, 10)

        #Search For Person
        self.search_for_person_publisher = self.create_publisher(SearchForPerson, 'search_for_person', 10)
        self.search_for_person_subscriber = self.create_subscription(ListOfPoints, "search_for_person_points", self.search_for_person_point_callback, 10)

        self.check_stickler_rules_publisher = self.create_publisher(Bool, "check_stickler_rules", 10)
        self.cropped_image_object_detected_subscriber = self.create_subscription(ListOfStrings, '/cropped_image_object_detected', self.list_of_stickler_rules_callback, 10)

        #RGB
        self.rgb_ctr = 2
        self.rgb = Int16()

        # Changing Variables
        self.state = 0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0

        self.flag_search_for_person_done = False
        self.start_button_state = False
        self.flag_person_inside_forbidden_room = True
        self.flag_rule_being_broken = False

        self.rule_unbroken = 100

        # Casa LAR
        self.begin_coordinates = (0.0, 3.0) # <----- CHANGE ME
        
        #self.forbidden_room_coordinates = (2.0, 6.9)
        #self.kitchen_room_coordinates = (0.5, 6.9) # coordenada da cozinha na porta entre cozinha e quarto
        #self.hall_room_coordinates = (1.3, 3.5)
        #self.living_room_coordinates = (1.3, 3.5)
        #self.entrance_coordinates = (0.0, 0.0)  
        #self.entrance_coordinates_orientation= (0.0, 0.0) 
        #self.bin_coordinates_orientation= (0.0, 0.0) # ----------> CHANGE ME
        #self.find_coordinates = (1.3 , 8.5)

        self.talk_neck = NeckPosition()
        self.talk_neck.pan = 180.0
        self.talk_neck.tilt = 180.0

        self.turn_around_neck = NeckPosition()
        self.turn_around_neck.pan = 360.0
        self.turn_around_neck.tilt = 180.0

        self.look_down = NeckPosition()
        self.look_down.pan = 180.0
        self.look_down.tilt = 165.0
        
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

        self.br = CvBridge()

        self.flag_left= False
        self.flag_right= False


        self.flag_first_time=True
        self.flag_speech_done = False
        self.flag_navigation_done = False
        self.flag_audio_done = False
        # self.keyword_list = []
        self.person_forbidden_room = 0
        self.person_with_shoes = 0
        self.person_with_drink = 0


        self.flag_shoes = False
        self.flag_drink = False
        self.flag_forbidden = False
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
        self.customer_position = []
        
        self.width = 0
        self.height = 0
        
        self.person_detected = Pose2D()
        self.person_detected.theta = -1.0


        self.Initial_state = 0
        self.Forbidden_room = 1
        self.Kitchen = 2
        self.BedRoom = 3 
        self.Corridor = 4
        self.LivingRoom = 5
        self.end = 99

        self.rule_drink = 0
        self.rule_garbage = 1
        self.rule_shoes = 2

        self.yolo_poses = Yolov8Pose()

        self.latest_stickler_rules_detected = ListOfStrings()
        self.flag_stckler_rules_done = False

    def list_of_stickler_rules_callback(self, los: ListOfStrings):
        self.latest_stickler_rules_detected = los
        self.flag_stckler_rules_done = True
     
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

    def yolov8_pose_callback(self, yolov8: Yolov8Pose):

        #self.get_logger().info('Receiving Yolov8 Pose Info')
        """ if(yolov8.persons[0].kp_nose_y != None):
            self.nose = yolov8.persons[0].kp_nose_y

        else:
            self.nose = self.int_error """
        

        self.yolo_poses = yolov8
        #self.person.append(yolov8) -> aqui queria tentar basicamente criar um conjunto de pessoas detetadas para depois olhar para o nariz delas

    def get_speech_done_callback(self, state: Bool):
        # print("Received Speech Flag:", state.data)
        self.flag_speech_done = True
        #self.start_audio()

    def search_for_person_point_callback(self, LoP: ListOfPoints):
        self.person_list_of_points = LoP
        self.flag_search_for_person_done = True
    
    def flag_pos_reached_callback(self, state: Bool):
        #print("Received Navigation Flag:", state.data)
        self.flag_navigation_done = True
    
    def start_audio(self):
        self.audio_command_publisher.publish(self.speech_type)

    def get_odometry_robot_callback(self, odom:Odometry):
        self.robot_current_position = odom

    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        print("Received Start Button:", state.data)

    def get_localisation_robot_callback(self, loc:Odometry):
        self.robot_x = loc.pose.pose.position.x
        self.robot_y = loc.pose.pose.position.y

        qx = loc.pose.pose.orientation.x
        qy = loc.pose.pose.orientation.y
        qz = loc.pose.pose.orientation.z
        qw = loc.pose.pose.orientation.w

        self.robot_t = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

    def check_if_person_left_forbidden_room(self):
        # Quando a pessoa sair, colocar flag_person_inside_forbidden_room = False
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SFTRNode()
    th_main = threading.Thread(target=thread_main_SFTR, args=(node,), daemon=True)
    th_main.start()
    rclpy.spin(node)
    rclpy.shutdown()


def thread_main_SFTR(node: SFTRNode):
    main = SFTRMain(node)
    main.main()


class SFTRMain():
    
    def __init__(self, node: SFTRNode):
        self.node = node
        self.start = time.time()
        self.end = time.time()
        i = 0
        
        
        
    def wait_for_end_of_speaking(self):
        while not self.node.flag_speech_done:
            pass
        self.node.flag_speech_done = False

    
    def wait_for_end_of_navigation(self):

        self.node.neck_position_publisher.publish(self.node.look_down)

        self.node.flag_navigation_done = False
        self.node.rgb_ctr = 66
        self.node.rgb.data = self.node.rgb_ctr
        self.node.rgb_mode_publisher.publish(self.node.rgb)

        self.node.rgb_ctr = 42
        self.node.rgb.data = self.node.rgb_ctr
        self.node.rgb_mode_publisher.publish(self.node.rgb)
        print("Started wait for end of navigation")
        while not self.node.flag_navigation_done:
            pass
        print("Finished wait for end of navigation")
        self.node.flag_navigation_done = False
        self.node.rgb_ctr = 62
        self.node.rgb.data = self.node.rgb_ctr
        self.node.rgb_mode_publisher.publish(self.node.rgb)
        time.sleep(1)
        print("Finished Navigation")

    def wait_for_end_of_rule_reception(self):
        while not self.node.flag_stckler_rules_done:
            pass
        self.node.flag_stckler_rules_done = False

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

    def wait_for_end_of_search_for_person(self):
        while not self.node.flag_search_for_person_done:
            pass
        self.node.flag_search_for_person_done = False
        print("Finished search for person")

    def choose_consequence(self, rule_unbroken):
        if rule_unbroken == self.node.rule_drink:

            self.node.rgb_ctr = 102 #tinoni
            self.node.rgb.data = self.node.rgb_ctr
            self.node.rgb_mode_publisher.publish(self.node.rgb)

            self.node.speech_str.command = "I detected a guest breaking the mandatory drink rule."
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            # OLHAR PARA PESSOA

            """ self.node.speech_str.command = "I am looking at the detected guest breaking the mandatory drink rule." 
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()
            time.sleep(0.5) # this is just so the looking at customers is not oo quick """

            ### FAzer eye contact com o guest
            
            track = TrackPerson()
            track.is_center = False
            track.kp_number = 0 # 0 is for nose
            track.person = self.node.yolo_poses.persons[0]
            self.node.neck_follow_person_publisher.publish(track)

            time.sleep(1) #para dar tempo de pescoço fazer movimento

            self.node.speech_str.command = "Hello there guest. You are breaking the mandatory drink rule."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.node.speech_str.command = "In order to be at this party, you must have a drink in your hand. Please follow me to the living room."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.node.speech_str.command = "I will turn around for you to follow me to the drink table. Don't try to trick me."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.wait_for_end_of_navigation()

            room = self.node.LivingRoom
            self.check_if_charmie_is_being_followed()

            self.node.speech_str.command = "Please wait here while I turn to you."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()
            
            self.wait_for_end_of_navigation()
       
            self.node.state = self.node.LivingRoom  

        elif rule_unbroken == self.node.rule_garbage:

            self.node.rgb_ctr = 102 #tinoni
            self.node.rgb.data = self.node.rgb_ctr
            self.node.rgb_mode_publisher.publish(self.node.rgb)

            self.node.speech_str.command = "I detected a guest breaking the no garbage on the floor rule."
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()
            
            """ # OLHAR PARA PESSOA

            self.node.speech_str.command = "I am looking at the detected guest breaking the no garbage on the floor rule." 
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()
            time.sleep(0.5) # this is just so the looking at customers is not oo quick """

            ### FAzer eye contact com o guest
            track = TrackPerson()
            track.is_center = False
            track.kp_number = 0 # 0 is for nose
            track.person = self.node.yolo_poses.persons[0]
            self.node.neck_follow_person_publisher.publish(track)
            time.sleep(1)

            self.node.speech_str.command = "Hello there guest. You are breaking the no garbage on the floor rule. "               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()


            self.node.speech_str.command = "In order to be at this party, you can't have garbage on the floor. Please follow me to the kitchen so you can put the garbage on the garbage bin."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.node.speech_str.command = "I will turn around for you to follow me to the kitchen. Don't try to trick me."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.wait_for_end_of_navigation()

            room = self.node.Kitchen
            self.check_if_charmie_is_being_followed()

            self.node.speech_str.command = "Please wait here while I turn to you."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()
            
            self.wait_for_end_of_navigation()
            
            self.node.state = self.node.end



        elif rule_unbroken == self.node.rule_shoes:

            self.node.rgb_ctr = 102 #tinoni
            self.node.rgb.data = self.node.rgb_ctr
            self.node.rgb_mode_publisher.publish(self.node.rgb)

            self.node.speech_str.command = "I detected a guest breaking the no shoes allowed rule."
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()
   
            # OLHAR PARA PESSOA

            """ self.node.speech_str.command = "I am looking at the detected guest breaking the no shoes allowed." 
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()
            time.sleep(0.5) # this is just so the looking at customers is not oo quick """

            track = TrackPerson()
            track.is_center = False
            track.kp_number = 0 # 0 is for nose
            track.person = self.node.yolo_poses.persons[0]
            self.node.neck_follow_person_publisher.publish(track)

            time.sleep(1) #para dar tempo de pescoço fazer movimento

            self.node.speech_str.command = "Hello there guest. You are breaking the no shoes allowed. "               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.node.speech_str.command = "In order to be at this party, you can't wear shoes. Please follow me to the entrance so you can take your shoes off."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.node.speech_str.command = "I will turn around for you to follow me to the entrance. Don't try to trick me."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.wait_for_end_of_navigation()

            room = self.node.LivingRoom
            self.check_if_charmie_is_being_followed()

            self.node.speech_str.command = "Please wait here while I turn to you."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()
            
            self.wait_for_end_of_navigation()

            self.node.state = self.node.BedRoom
     

    def check_rule_drink(self):
        while self.node.flag_rule_being_broken == True :
            self.node.speech_str.command = "Please take any drink from the shelf."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.sleep(5)

            self.node.speech_str.command = "Please stand in front of me so I can analise you again."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            # Funçao Tiago de análise
            # Caso a regra deixe de ser cumprida, retorno flag = False 

        
    def check_rule_garbage(self):
        while self.node.flag_rule_being_broken == True :
            self.node.speech_str.command = "Please place the garbage at the garbage bin."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.sleep(5)

            self.node.speech_str.command = "Please stand in front of me so I can analise you again."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            # Funçao Tiago de análise
            # Caso a regra deixe de ser cumprida, retorno flag = False 
        

    def check_rule_shoes(self):
        while self.node.flag_rule_being_broken == True :
            self.node.speech_str.command = "Please place your shoes on the shelf."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            self.sleep(5)

            self.node.speech_str.command = "Please stand in front of me so I can analise you again."               
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()

            # Funçao Tiago de análise
            # Caso a regra deixe de ser cumprida, retorno flag = False 


    
    def main(self):
        time.sleep(1)
        print("IN NEW MAIN")
        pose = Bool()
        pose.data = False
        self.node.only_detect_person_right_in_front_publisher.publish(pose)

        
        while True:

            # primeiro estado será anunciar que está pronto, ligar luzes, e ir para forbidden room
            # segundo será fazer tracking + verificar pessoa + anunciar que há uma regra a ser quebrada + olhar para pessoa + dizer lhe que está a quebrar a regra
            # + virar para fora da sala + caminhar para porta que liga a próxima divisão + virar cabeça para trás + verificar se pessoa está a seguir 
            # (aqui loop caso n esteja a dizer para a pessoa o seguir enquanto caminha até um pouco fora da sala de forma a que a pessoa o possa seguir e eu ver)
            # quando confirmar que a pessoa saiu, ir para o próximo estado (idealmente também havia um timeout, mas vou só fazer assim enquanto)


            # No terceiro estado ir para o canto de uma divisão (cozinha) e fazer tracking. Guardar coordenadas das pessoas detetadas, olhar para elas, verificar
            # se estão na divisão, as que estão guardar as suas posições, ir na direção delas, usar código de tirar fotos ao corpo todo e analisar o que tem ao lado
            # Sempre associado a este código tenho de ter uma função que recebe a regra que está a quebrar (se for sapatos, envia a pessoa para a entrada, se for lixo
            # acompanha ao lixo, se for bebidas acompanha à sala. No final verifica se havia mais pessoas por verificar na divisão onde estava (pq guardou o nr de pessoas
            # a verificar) e se hpouvesse volta para lá e vai ter com elas, senão vai para a divisão seguinte). E assim sucessivamente 
    
            #Navegação até ao espaço proibido
            if self.node.state == self.node.Initial_state:
                print('Initial position state')

                self.node.neck_position_publisher.publish(self.node.talk_neck)
                
                self.node.rgb_ctr = 100
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb)

                self.node.get_logger().info("Initial position state")

                
                ########## EXEMPLO DO FOLLOW PERSON COM O NECK
                """ track = TrackPerson()
                while True:
                    if len(self.node.yolo_poses.persons) > 0:
                        print("Person Found")

                        track.is_center = False
                        track.kp_number = 0 # 0 is for nose
                        track.person = self.node.yolo_poses.persons[0]
                        self.node.neck_follow_person_publisher.publish(track)

                        self.node.speech_str.command = "Yes."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking() 
                    else:
                        print("Person not Found")
                        self.node.speech_str.command = "No."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking() 
                    time.sleep(3)
                ### RESUMIDO SO PRECISAS DISTO:
                #     track = TrackPerson()
                #     track.is_center = False
                #     track.kp_number = 0 # 0 is for nose
                #     track.person = self.node.yolo_poses.persons[0]
                #     self.node.neck_follow_person_publisher.publish(track) """
               

                ########## EXEMPLO DE PESSOAS A SEGUIR CHARMIE, DEPOIS TEM QUE SER ADAPTADO PARA FICAR NO PERSON RECOGNITION MAS PARA JA FICA ASSIM
                # self.check_if_charmie_is_being_followed()


                """ ########## EXEMPLO DE VERIFICAR REGRAS

                self.node.neck_position_publisher.publish(self.node.look_down)
                flag_check_rules = Bool()
                while True:
                    if len(self.node.yolo_poses.persons) > 0:
                        print("Person Found")

                        flag_check_rules.data = True
                        self.node.check_stickler_rules_publisher.publish(flag_check_rules)
                        self.wait_for_end_of_rule_reception()

                        print(self.node.latest_stickler_rules_detected)

                        check_drink_hand, check_shoes, check_trash =  self.analise_stickler_rules(self.node.latest_stickler_rules_detected)
                        print("Drink in Hand:", check_drink_hand, "Shoes:", check_shoes, "Trash:", check_trash)



                        self.node.speech_str.command = "Yes."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking() 
                    else:
                        # print("Person not Found")
                        self.node.speech_str.command = "No."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking() 
                    time.sleep(5) """

                self.node.speech_str.command = "Hello! I am ready to start the Stickler for the rules task! Waiting for start button to be pressed"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking() 

                t = Bool()
                t.data = True
                self.node.flag_start_button_publisher.publish(t)
                self.wait_for_start_button()
                
                self.node.rgb_ctr = 24
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                self.node.rgb_ctr = 41
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                # Robot deslocar-se até entrada da forbidden room
                # self.wait_for_end_of_navigation()

                self.node.state = self.node.Forbidden_room

            elif self.node.state == self.node.Forbidden_room:
            
                self.node.speech_str.command = "I will start by checking if there is someone breaking the forbidden room rule."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                # ir até entrada
                self.wait_for_end_of_navigation()

                self.node.speech_str.command = "Start tracking."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                sfp = SearchForPerson()
                sfp.angles = [-45, 0, 45]
                sfp.show_image_people_detected = True
                self.node.search_for_person_publisher.publish(sfp)
                self.wait_for_end_of_search_for_person()

                self.node.neck_position_publisher.publish(self.node.talk_neck)

                # just for debug purposes, it has its own switch case so it can be done right after receving and not wait for neck
                if len(self.node.person_list_of_points.coords) == 0:
                    self.node.rgb_ctr = 0
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)
                    self.node.speech_str.command = "I did not detect anyone breaking the forbidden room rule."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()
                    self.node.state = self.node.Kitchen

                elif len(self.node.person_list_of_points.coords) == 1:

                    self.node.rgb_ctr = 102 #tinoni
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)
                    
                    self.node.rgb_ctr = 40
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)
                    self.node.speech_str.command = "I detected a guest breaking the forbidden room rule."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    for p in self.node.person_list_of_points.coords:
                        pose = Pose2D()
                        pose.x = p.x
                        pose.y = p.y                
                        print('a')
                        self.node.customer_position.append(pose)
                        pose.theta = 180.0
                        print('b')
                        self.node.neck_to_coords_publisher.publish(pose)
                        #time.sleep(1) # this is just so the looking at customers is not oo quick
                        


                    """ ### FAzer eye contact com o guest
                    track = TrackPerson()
                    track.is_center = False
                    track.kp_number = 0 # 0 is for nose
                    track.person = self.node.yolo_poses.persons[0]
                    self.node.neck_follow_person_publisher.publish(track)
                    time.sleep(1) """

                    self.node.speech_str.command = "I am looking at the detected guest breaking the forbidden room rule." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.speech_str.command = "Hello there guest. You are breaking the forbidden room rule. "               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()


                    self.node.speech_str.command = "To stop breaking this rule, you must get out of the office and join the other guests in the other rooms of the house."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.speech_str.command = "I will turn around for you to follow me. Don't try to trick me."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    # Robot orienta-se para fora da forbidden room (ficando na zona da porta)
                    self.wait_for_end_of_navigation()

                    self.node.neck_position_publisher.publish(self.node.turn_around_neck)

                    time.sleep(1)

                    self.check_if_charmie_is_being_followed()

                    self.node.rgb_ctr = 14
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 

                    # the person stopped following the robot
                    self.node.speech_str.command = "You are no longer breaking the rule. Keep enjoying the party without breaking any rules."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.neck_position_publisher.publish(self.node.talk_neck)

                    self.node.state = self.node.Kitchen
                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 

                    time.sleep(1)

                else:
                    self.node.speech_str.command = "I detected more than 1 guest breaking the rule. I am not prepared to solve that yet."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.rgb_ctr = 10
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb)
                    
                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 

            # No terceiro estado ir para o canto de uma divisão (cozinha) e fazer tracking. Guardar coordenadas das pessoas detetadas, olhar para elas, verificar
            # se estão na divisão, as que estão guardar as suas posições, ir na direção delas, usar código de tirar fotos ao corpo todo e analisar o que tem ao lado
            # Sempre associado a este código tenho de ter uma função que recebe a regra que está a quebrar (se for sapatos, envia a pessoa para a entrada, se for lixo
            # acompanha ao lixo, se for bebidas acompanha à sala. No final verifica se havia mais pessoas por verificar na divisão onde estava (pq guardou o nr de pessoas
            # a verificar) e se hpouvesse volta para lá e vai ter com elas, senão vai para a divisão seguinte). E assim sucessivamente 

            elif self.node.state == self.node.Kitchen:
                
                self.node.speech_str.command = "I will check if there is someone breaking rules in the kitchen."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                pose = Bool()
                pose.data = False
                self.node.only_detect_person_right_in_front_publisher.publish(pose)

                # Robot deslocar-se até saída da cozinha e orienta-se para dentro da cozinha
                self.wait_for_end_of_navigation()

                self.node.speech_str.command = "Start tracking."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                sfp = SearchForPerson()
                sfp.angles = [-45, 0, 45]
                sfp.show_image_people_detected = True
                self.node.search_for_person_publisher.publish(sfp)
                self.wait_for_end_of_search_for_person()

                nr_persons_division = len(self.node.person_list_of_points.coords)

                self.i = 0

                if nr_persons_division == 0:
                    self.node.speech_str.command = "I didn't detect anyone in the kitchen."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.state = self.node.BedRoom

                else:

                    self.node.rgb_ctr = 12
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 

                    self.node.speech_str.command = f"I detected " + str(nr_persons_division) + " persons in the kitchen."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 
                    

                    p_ctr = 0
                    for p in self.node.person_list_of_points.coords:
                        p_ctr+=1 # just to count the people detected and add that info to the string that the robot will speak
                        pose = Pose2D()
                        pose.x = p.x
                        pose.y = p.y
                        self.node.customer_position.append(pose)
                        pose.theta = 180.0
                        self.node.neck_to_coords_publisher.publish(pose)
                    
                    
                    ### FAzer eye contact com o guest
                    # track = TrackPerson()
                    # track.is_center = False
                    # track.kp_number = 0 # 0 is for nose
                    # track.person = self.node.yolo_poses.persons[0]
                    # self.node.neck_follow_person_publisher.publish(track)
                    #time.sleep(1)

                    self.node.speech_str.command = "I am looking at the guest." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.speech_str.command = "Moving to the guest." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.wait_for_end_of_navigation()

                    self.node.neck_position_publisher.publish(self.node.look_down)
                    flag_check_rules = Bool()
                
                    if len(self.node.yolo_poses.persons) > 0:
                        print("Person Found")

                        self.node.rgb_ctr = 55
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        flag_check_rules.data = True
                        self.node.check_stickler_rules_publisher.publish(flag_check_rules)
                        self.wait_for_end_of_rule_reception()

                        print(self.node.latest_stickler_rules_detected)

                        check_drink_hand, check_shoes, check_trash =  self.analise_stickler_rules(self.node.latest_stickler_rules_detected)
                        print("Drink in Hand:", check_drink_hand, "Shoes:", check_shoes, "Trash:", check_trash)

                            # Chamar função que recebe regra a ser quebrada e que faz com que o robot haja em conformidade com essa regra
                        #if check_drink_hand == True:
                        
                        self.choose_consequence(self.node.rule_drink) 
                        self.node.speech_str.command = "Please take a drink from the drink table and then place in front of me so I can check if you are following the rule." 
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()

                        """ elif check_trash == True:
                            self.choose_consequence(self.node.rule_garbage) 
                        elif check_shoes == True:
                            self.choose_consequence(self.node.rule_shoes)
                        else:

                            self.node.speech_str.command = "You are a saint. No rules being broken." 
                            self.node.speaker_publisher.publish(self.node.speech_str)
                            self.wait_for_end_of_speaking() """
                        

                        self.node.rgb_ctr = 34
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        check_drink_hand = True

                        while check_drink_hand == True or check_trash == True or check_shoes == True:
                        
                            time.sleep(5)

                            self.node.rgb_ctr = 55
                            self.node.rgb.data = self.node.rgb_ctr
                            self.node.rgb_mode_publisher.publish(self.node.rgb) 

                            pose = Bool()
                            pose.data = True
                            self.node.only_detect_person_right_in_front_publisher.publish(pose)

                            flag_check_rules.data = True
                            self.node.check_stickler_rules_publisher.publish(flag_check_rules)
                            self.wait_for_end_of_rule_reception()

                            print(self.node.latest_stickler_rules_detected)



                            if len(self.node.yolo_poses.persons) > 0:                                
                                check_drink_hand, check_shoes, check_trash =  self.analise_stickler_rules(self.node.latest_stickler_rules_detected)
                           
                            else:
                                self.node.speech_str.command = "Please stand in front of me so I can check if you stopped breaking the rule." 
                                self.node.speaker_publisher.publish(self.node.speech_str)
                                self.wait_for_end_of_speaking()

                            self.node.speech_str.command = "Please take a drink from the drinking table and show it to me in a clear way." 
                            self.node.speaker_publisher.publish(self.node.speech_str)
                            self.wait_for_end_of_speaking()


                        track = TrackPerson()
                        track.is_center = False
                        track.kp_number = 0 # 0 is for nose
                        track.person = self.node.yolo_poses.persons[0]
                        self.node.neck_follow_person_publisher.publish(track)

                        pose = Bool()
                        pose.data = False
                        self.node.only_detect_person_right_in_front_publisher.publish(pose)

                        self.node.rgb_ctr = 24
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        self.node.speech_str.command = "You are no longer breaking the rule. Keep enjoying the party without breaking any rules."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                    else:
                        # print("Person not Found")
                        self.node.speech_str.command = "No."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking() 
                    
                    time.sleep(1)
        
                    self.i += 1

            elif self.node.state == self.node.BedRoom:
                
                self.node.speech_str.command = "I will check if there is someone breaking rules in the bedroom."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                pose = Bool()
                pose.data = False
                self.node.only_detect_person_right_in_front_publisher.publish(pose)

                # Robot deslocar-se até saída da cozinha e orienta-se para dentro da cozinha
                self.wait_for_end_of_navigation()

                self.node.speech_str.command = "Start tracking."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                sfp = SearchForPerson()
                sfp.angles = [-45, 0, 45]
                sfp.show_image_people_detected = True
                self.node.search_for_person_publisher.publish(sfp)
                self.wait_for_end_of_search_for_person()

                nr_persons_division = len(self.node.person_list_of_points.coords)

                self.i = 0

                if nr_persons_division == 0:
                    self.node.speech_str.command = "I didn't detect anyone in the bedroom."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.state = self.node.Corridor

                else:

                    self.node.rgb_ctr = 12
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 

                    self.node.speech_str.command = f"I detected " + str(nr_persons_division) + " persons in the bedroom."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 
                    

                    p_ctr = 0
                    for p in self.node.person_list_of_points.coords:
                        p_ctr+=1 # just to count the people detected and add that info to the string that the robot will speak
                        pose = Pose2D()
                        pose.x = p.x
                        pose.y = p.y
                        self.node.customer_position.append(pose)
                        pose.theta = 180.0
                        self.node.neck_to_coords_publisher.publish(pose)
                    
                    
                    ### FAzer eye contact com o guest
                    # track = TrackPerson()
                    # track.is_center = False
                    # track.kp_number = 0 # 0 is for nose
                    # track.person = self.node.yolo_poses.persons[0]
                    # self.node.neck_follow_person_publisher.publish(track)
                    #time.sleep(1)

                    self.node.speech_str.command = "I am looking at the guest." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.speech_str.command = "Moving to the guest." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.wait_for_end_of_navigation()

                    self.node.neck_position_publisher.publish(self.node.look_down)
                    flag_check_rules = Bool()
                
                    if len(self.node.yolo_poses.persons) > 0:
                        print("Person Found")

                        self.node.rgb_ctr = 55
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        flag_check_rules.data = True
                        self.node.check_stickler_rules_publisher.publish(flag_check_rules)
                        self.wait_for_end_of_rule_reception()

                        print(self.node.latest_stickler_rules_detected)

                        check_drink_hand, check_shoes, check_trash =  self.analise_stickler_rules(self.node.latest_stickler_rules_detected)
                        print("Drink in Hand:", check_drink_hand, "Shoes:", check_shoes, "Trash:", check_trash)

                            # Chamar função que recebe regra a ser quebrada e que faz com que o robot haja em conformidade com essa regra
                        #if check_drink_hand == True:
                        #    self.choose_consequence(self.node.rule_drink) 
                        #elif check_trash == True:
                        self.choose_consequence(self.node.rule_garbage) 
                        self.node.speech_str.command = "Please place the garbage at the garbage bin and then place in front of me so I can check if you are following the rule." 
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                        #elif check_shoes == True:
                        #    self.choose_consequence(self.node.rule_shoes)

                        """ else:
                            self.node.speech_str.command = "You are a saint. No rules being broken." 
                            self.node.speaker_publisher.publish(self.node.speech_str)
                            self.wait_for_end_of_speaking() """

                        self.node.rgb_ctr = 34
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        while check_drink_hand == True or check_trash == True or check_shoes == True:
                        
                            time.sleep(5)

                            self.node.rgb_ctr = 55
                            self.node.rgb.data = self.node.rgb_ctr
                            self.node.rgb_mode_publisher.publish(self.node.rgb) 

                            pose = Bool()
                            pose.data = True
                            self.node.only_detect_person_right_in_front_publisher.publish(pose)

                            flag_check_rules.data = True
                            self.node.check_stickler_rules_publisher.publish(flag_check_rules)
                            self.wait_for_end_of_rule_reception()

                            print(self.node.latest_stickler_rules_detected)

                            if len(self.node.yolo_poses.persons) > 0:                                
                                check_drink_hand, check_shoes, check_trash =  self.analise_stickler_rules(self.node.latest_stickler_rules_detected)
                           
                            else:
                                self.node.speech_str.command = "Please stand in front of me so I can check if you stopped breaking the rule." 
                                self.node.speaker_publisher.publish(self.node.speech_str)
                                self.wait_for_end_of_speaking()

                        track = TrackPerson()
                        track.is_center = False
                        track.kp_number = 0 # 0 is for nose
                        track.person = self.node.yolo_poses.persons[0]
                        self.node.neck_follow_person_publisher.publish(track)

                        pose = Bool()
                        pose.data = False
                        self.node.only_detect_person_right_in_front_publisher.publish(pose)

                        self.node.rgb_ctr = 24
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        self.node.speech_str.command = "You are no longer breaking the rule. Keep enjoying the party without breaking any rules."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                    else:
                        # print("Person not Found")
                        self.node.speech_str.command = "No."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking() 
                    time.sleep(5)
        
                    self.i += 1

            elif self.node.state == self.node.Corridor:
                
                self.node.speech_str.command = "I will check if there is someone breaking rules in the corridor."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                pose = Bool()
                pose.data = False
                self.node.only_detect_person_right_in_front_publisher.publish(pose)

                # Robot deslocar-se até saída da cozinha e orienta-se para dentro da cozinha
                self.wait_for_end_of_navigation()

                self.node.speech_str.command = "Start tracking."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                sfp = SearchForPerson()
                sfp.angles = [-45, 0, 45]
                sfp.show_image_people_detected = True
                self.node.search_for_person_publisher.publish(sfp)
                self.wait_for_end_of_search_for_person()

                nr_persons_division = len(self.node.person_list_of_points.coords)

                self.i = 0

                if nr_persons_division == 0:
                    self.node.speech_str.command = "I didn't detect anyone in the corridor."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.state = self.node.LivingRoom

                else:

                    self.node.rgb_ctr = 12
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 

                    self.node.speech_str.command = f"I detected " + str(nr_persons_division) + " persons in the corridor."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 
                    

                    p_ctr = 0
                    for p in self.node.person_list_of_points.coords:
                        p_ctr+=1 # just to count the people detected and add that info to the string that the robot will speak
                        pose = Pose2D()
                        pose.x = p.x
                        pose.y = p.y
                        self.node.customer_position.append(pose)
                        pose.theta = 180.0
                        self.node.neck_to_coords_publisher.publish(pose)
                    
                    
                    ### FAzer eye contact com o guest
                    # track = TrackPerson()
                    # track.is_center = False
                    # track.kp_number = 0 # 0 is for nose
                    # track.person = self.node.yolo_poses.persons[0]
                    # self.node.neck_follow_person_publisher.publish(track)
                    #time.sleep(1)

                    self.node.speech_str.command = "I am looking at the guest." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.speech_str.command = "Moving to the guest." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.wait_for_end_of_navigation()

                    self.node.neck_position_publisher.publish(self.node.look_down)
                    flag_check_rules = Bool()
                
                    if len(self.node.yolo_poses.persons) > 0:
                        print("Person Found")

                        self.node.rgb_ctr = 55
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        flag_check_rules.data = True
                        self.node.check_stickler_rules_publisher.publish(flag_check_rules)
                        self.wait_for_end_of_rule_reception()

                        print(self.node.latest_stickler_rules_detected)

                        check_drink_hand, check_shoes, check_trash =  self.analise_stickler_rules(self.node.latest_stickler_rules_detected)
                        print("Drink in Hand:", check_drink_hand, "Shoes:", check_shoes, "Trash:", check_trash)

                            # Chamar função que recebe regra a ser quebrada e que faz com que o robot haja em conformidade com essa regra
                        #if check_drink_hand == True:
                        #    self.choose_consequence(self.node.rule_drink) 
                        #elif check_trash == True:
                        #    self.choose_consequence(self.node.rule_garbage) 
                        #elif check_shoes == True:
                        self.choose_consequence(self.node.rule_shoes)





                        self.node.speech_str.command = "Please take off your shoes and then place in front of me so I can check if you are following the rule." 
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()

                        """ else:
                            self.node.speech_str.command = "You are a saint. No rules being broken." 
                            self.node.speaker_publisher.publish(self.node.speech_str)
                            self.wait_for_end_of_speaking() """
                        
                        self.node.rgb_ctr = 34
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        while check_drink_hand == True or check_trash == True or check_shoes == True:
                        
                            time.sleep(5)

                            self.node.rgb_ctr = 55
                            self.node.rgb.data = self.node.rgb_ctr
                            self.node.rgb_mode_publisher.publish(self.node.rgb) 

                            pose = Bool()
                            pose.data = True
                            self.node.only_detect_person_right_in_front_publisher.publish(pose)

                            flag_check_rules.data = True
                            self.node.check_stickler_rules_publisher.publish(flag_check_rules)
                            self.wait_for_end_of_rule_reception()

                            print(self.node.latest_stickler_rules_detected)

                            if len(self.node.yolo_poses.persons) > 0:                                
                                check_drink_hand, check_shoes, check_trash =  self.analise_stickler_rules(self.node.latest_stickler_rules_detected)
                           
                            else:
                                self.node.speech_str.command = "Please stand in front of me so I can check if you stopped breaking the rule." 
                                self.node.speaker_publisher.publish(self.node.speech_str)
                                self.wait_for_end_of_speaking()

                        track = TrackPerson()
                        track.is_center = False
                        track.kp_number = 0 # 0 is for nose
                        track.person = self.node.yolo_poses.persons[0]
                        self.node.neck_follow_person_publisher.publish(track)

                        pose = Bool()
                        pose.data = False
                        self.node.only_detect_person_right_in_front_publisher.publish(pose)

                        self.node.rgb_ctr = 24
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        self.node.speech_str.command = "You are no longer breaking the rule. Keep enjoying the party without breaking any rules."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()
                    else:
                        # print("Person not Found")
                        self.node.speech_str.command = "No."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking() 
                    time.sleep(5)
        
                    self.i += 1

            elif self.node.state == self.node.LivingRoom:

                self.node.speech_str.command = "I will check if there is someone breaking rules in the living room."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                pose = Bool()
                pose.data = False
                self.node.only_detect_person_right_in_front_publisher.publish(pose)

                # Robot deslocar-se até saída da cozinha e orienta-se para dentro da cozinha
                self.wait_for_end_of_navigation()

                self.node.speech_str.command = "Start tracking."               
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                sfp = SearchForPerson()
                sfp.angles = [-45, 0, 45]
                sfp.show_image_people_detected = True
                self.node.search_for_person_publisher.publish(sfp)
                self.wait_for_end_of_search_for_person()

                nr_persons_division = len(self.node.person_list_of_points.coords)

                self.i = 0

                if nr_persons_division == 0:
                    self.node.speech_str.command = "I didn't detect anyone in the living room."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.state = self.node.Corridor

                else:

                    self.node.rgb_ctr = 12
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 

                    self.node.speech_str.command = f"I detected " + str(nr_persons_division) + " persons in the living room."               
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.rgb_ctr = 24
                    self.node.rgb.data = self.node.rgb_ctr
                    self.node.rgb_mode_publisher.publish(self.node.rgb) 
                    

                    p_ctr = 0
                    for p in self.node.person_list_of_points.coords:
                        p_ctr+=1 # just to count the people detected and add that info to the string that the robot will speak
                        pose = Pose2D()
                        pose.x = p.x
                        pose.y = p.y
                        self.node.customer_position.append(pose)
                        pose.theta = 180.0
                        self.node.neck_to_coords_publisher.publish(pose)
                    
                    
                    ### FAzer eye contact com o guest
                    # track = TrackPerson()
                    # track.is_center = False
                    # track.kp_number = 0 # 0 is for nose
                    # track.person = self.node.yolo_poses.persons[0]
                    # self.node.neck_follow_person_publisher.publish(track)
                    #time.sleep(1)

                    self.node.speech_str.command = "I am looking at the guest." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.node.speech_str.command = "Moving to the guest." 
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()

                    self.wait_for_end_of_navigation()

                    self.node.neck_position_publisher.publish(self.node.look_down)
                    flag_check_rules = Bool()
                
                    if len(self.node.yolo_poses.persons) > 0:
                        print("Person Found")

                        self.node.rgb_ctr = 55
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        flag_check_rules.data = True
                        self.node.check_stickler_rules_publisher.publish(flag_check_rules)
                        self.wait_for_end_of_rule_reception()

                        print(self.node.latest_stickler_rules_detected)

                        check_drink_hand, check_shoes, check_trash =  self.analise_stickler_rules(self.node.latest_stickler_rules_detected)
                        print("Drink in Hand:", check_drink_hand, "Shoes:", check_shoes, "Trash:", check_trash)

                        """     # Chamar função que recebe regra a ser quebrada e que faz com que o robot haja em conformidade com essa regra
                        if check_drink_hand == True:
                            self.choose_consequence(self.node.rule_drink) 
                        elif check_trash == True:
                            self.choose_consequence(self.node.rule_garbage) 
                        elif check_shoes == True:
                            self.choose_consequence(self.node.rule_shoes)
                        else: """
                        self.node.speech_str.command = "You are not breaking any rule. Keep enjoying the party with the other guests." 
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking()

                        self.node.rgb_ctr = 34
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        """ while check_drink_hand == True or check_trash == True or check_shoes == True:
                        
                            time.sleep(5)

                            self.node.rgb_ctr = 55
                            self.node.rgb.data = self.node.rgb_ctr
                            self.node.rgb_mode_publisher.publish(self.node.rgb) 

                            pose = Bool()
                            pose.data = True
                            self.node.only_detect_person_right_in_front_publisher.publish(pose)

                            flag_check_rules.data = True
                            self.node.check_stickler_rules_publisher.publish(flag_check_rules)
                            self.wait_for_end_of_rule_reception()

                            print(self.node.latest_stickler_rules_detected)

                            if len(self.node.yolo_poses.persons) > 0:                                
                                check_drink_hand, check_shoes, check_trash =  self.analise_stickler_rules(self.node.latest_stickler_rules_detected)
                           
                            else:
                                self.node.speech_str.command = "Please stand in front of me so I can check if you stopped breaking the rule." 
                                self.node.speaker_publisher.publish(self.node.speech_str)
                                self.wait_for_end_of_speaking() """

                        track = TrackPerson()
                        track.is_center = False
                        track.kp_number = 0 # 0 is for nose
                        track.person = self.node.yolo_poses.persons[0]
                        self.node.neck_follow_person_publisher.publish(track)

                        pose = Bool()
                        pose.data = False
                        self.node.only_detect_person_right_in_front_publisher.publish(pose)

                        self.node.rgb_ctr = 24
                        self.node.rgb.data = self.node.rgb_ctr
                        self.node.rgb_mode_publisher.publish(self.node.rgb) 

                        """ self.node.speech_str.command = "You are no longer breaking the rule. Keep enjoying the party without breaking any rules."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking() """

                        self.node.state = self.node.Corridor
                    else:
                        # print("Person not Found")
                        self.node.speech_str.command = "No."
                        self.node.speaker_publisher.publish(self.node.speech_str)
                        self.wait_for_end_of_speaking() 
                    
                    time.sleep(1)
                    self.node.state = self.node.Corridor
        
                    self.i += 1

            elif self.node.state == self.node.end:
                self.node.rgb_ctr = 100
                self.node.rgb.data = self.node.rgb_ctr
                self.node.rgb_mode_publisher.publish(self.node.rgb) 

                self.node.speech_str.command = "I've finished my stickler for the rules task."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()

                while True:
                    pass

    def check_if_charmie_is_being_followed(self):

        # sends info to yolo pose to only detect people right in front of the camera
        pose = Bool()
        pose.data = True
        self.node.only_detect_person_right_in_front_publisher.publish(pose)

        # looks back to check if is being followed 
        neck_look_back = NeckPosition()
        neck_look_back.pan = float(360)
        neck_look_back.tilt = float(180)
        self.node.neck_position_publisher.publish(neck_look_back)

        time.sleep(1.0)

        #self.node.speech_str.command = "Please Follow Me. Keep yourself approximately 1 meter behind me. If you start to get behind I will warn you"
        self.node.speech_str.command = "Please Follow Me."
        self.node.speaker_publisher.publish(self.node.speech_str)
        self.wait_for_end_of_speaking()

        person_here = False
        while not person_here:
            if len(self.node.yolo_poses.persons) > 0:
                self.node.speech_str.command = "Thanks for coming behind me. Let's roll."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                person_here = True
            else:
                # the person stopped following the robot
                self.node.speech_str.command = "Please come behind me. I need you to follow me."
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()
                time.sleep(3.0)


        prev_number_of_person = 1
        while not self.node.flag_navigation_done:
          

            if len(self.node.yolo_poses.persons) > 0:
                if prev_number_of_person == 0:
                    self.node.speech_str.command = "Thanks for coming back. Let's roll."
                    self.node.speaker_publisher.publish(self.node.speech_str)
                    self.wait_for_end_of_speaking()    
                    prev_number_of_person = 1   
                # everything is ok
            else:
                # the person stopped following the robot
                self.node.speech_str.command = "You are falling behind, please come back. I will wait here for you"
                self.node.speaker_publisher.publish(self.node.speech_str)
                self.wait_for_end_of_speaking()        
                prev_number_of_person = 0
                time.sleep(3.0)


        self.node.flag_navigation_done = False

        pose = Bool()
        pose.data = True
        self.node.only_detect_person_right_in_front_publisher.publish(pose)



    def analise_stickler_rules(self, rules: ListOfStrings):

        check_drink_hand = False
        check_shoes = False
        check_trash = False

        if rules.strings[0] == '' and rules.strings[1] == '':
            print("Breaking Drink in Hand Rule")
            check_drink_hand = True
            """ self.node.speech_str.command = "Breaking Drink in Hand Rule."
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()  """
        
        if rules.strings[3] == 'shoe' and rules.strings[4] == 'shoe':
            print("Breaking Shoes Rule")
            check_shoes = True
            """ self.node.speech_str.command = "Breaking Shoes Rule."
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()  """
        
        if rules.strings[2] != '':
            print("Breaking Garbage Rule")
            check_trash = True
            """ self.node.speech_str.command = "Breaking Garbage Rule."
            self.node.speaker_publisher.publish(self.node.speech_str)
            self.wait_for_end_of_speaking()  """

        return check_drink_hand, check_shoes, check_trash