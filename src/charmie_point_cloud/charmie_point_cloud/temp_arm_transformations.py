#!/usr/bin/env python3

# initial instructions:
# this is an example of the layout code for a node in our workspace
# It is being used Face as an example, so when changing the code for a specific task Ctrl+F face
# and replace everything fot the name of your node 

import rclpy
from rclpy.node import Node

# import variables from standard libraries and both messages and services from custom charmie_interfaces
from example_interfaces.msg import Bool, String, Int16
from charmie_interfaces.msg import SpeechType, RobotSpeech
from charmie_interfaces.srv import SpeechCommand

import numpy as np

import math

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_4  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class Face():
    def __init__(self):
        # initialised face class
        # all non ros2 functions that have to do with the face must be inside this class
        print("New Face Class Initialised")


class FaceNode(Node):
    def __init__(self):
        super().__init__("Face")
        self.get_logger().info("Initialised CHARMIE Face Node")

        # Initialise face class
        self.face = Face()

        ### Topics (Publisher and Subscribers) ###   
        # Low Level   
        self.rgb_mode_publisher = self.create_publisher(Int16, "rgb_mode", 10)    
        self.start_button_subscriber = self.create_subscription(Bool, "get_start_button", self.get_start_button_callback, 10)
        
        ### Services (Clients) ###
        # Speakers
        self.speech_command_client = self.create_client(SpeechCommand, "speech_command")

        # if is necessary to wait for a specific service to be ON, uncomment the two following lines
        # while not self.speech_command_client.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Speech Command...")
        
        # Variables 
        self.waited_for_end_of_speaking = False
        self.flag_navigation_done = False


    # Subscriber to get the start button status     
    def get_start_button_callback(self, state: Bool):
        self.start_button_state = state.data
        print("Received Start Button:", state.data)
        
    """ def deg_to_rad(self, deg):
        rad = [deg[0] * math.pi / 180]
        return rad """
        
    """ def Rot(self, axis: String, rotation: Int16):
        if axis == 'x':
            R = np.array([[1,                                    0,                                  0],
                          [0, math.cos(self.deg_to_rad(rotation)), -math.sin(self.deg_to_rad(rotation))],
                          [0, math.sin(self.deg_to_rad(rotation)), math.cos(self.deg_to_rad(rotation))]])
        elif axis == 'y':
            R = np.array([[ math.cos(self.deg_to_rad(rotation)), 0, math.sin(self.deg_to_rad(rotation))],
                          [0,                                    1,                                   0],
                          [-math.sin(self.deg_to_rad(rotation)), 0, math.cos(self.deg_to_rad(rotation))]])
        elif axis == 'z':
            R = np.array([[math.cos(self.deg_to_rad(rotation)), -math.sin(self.deg_to_rad(rotation)), 0],
                          [math.sin(self.deg_to_rad(rotation)), math.cos(self.deg_to_rad(rotation)), 0],
                          [0,                                    0,                                  1]])
        
        print(R)
        return R """
    
    def Rot(self, eixo, angulo):
        ang_rad = angulo*math.pi/180.0
        c = math.cos(ang_rad)
        s = math.sin(ang_rad)
        M = np.identity(4)
        if (eixo == 'x' or eixo == 'X'):
            M[1][1] = M[2][2] = c
            M[1][2] = -s
            M[2][1] = s
        elif (eixo == 'y' or eixo == 'Y'):
            M[0][0] = M[2][2] = c
            M[0][2] = s
            M[2][0] = -s
        elif (eixo == 'z' or eixo == 'Z'):
            M[0][0] = M[1][1] = c
            M[0][1] = -s
            M[1][0] = s
        return M
        
    def Trans(self, tx, ty, tz):
        M = np.identity(4)
        M[0][3] = tx
        M[1][3] = ty
        M[2][3] = tz
        return M


    def transform(self):
        # C representa o ponto no espaço para onde eu quero transformar a base do braço do robô
        # A matriz de transformação desde a  base do braço até ao centro do Robot pode ser representada por:
        # T = Rot(z, 180) * Rot (x, -90) * Trans (3, -6, -110)
        # a2 representa a translação desde a base do braço até ao centro do robô  (em cm)
        # a1 representa a rotação sobre o eixo coordenadas x em -90º para alinhar os eixos coordenados
        # a0 representa a rotação sobre o eixo coordenadas z em 180º para alinhar o eixo dos x 
        # c representa o ponto (x,y,z) em relação ao centro do braço

        ### x representa a frente do robô. y positivo vai para a esquerda do robô. z vai para cima no robô
        c = np.dot(np.identity(4), [0, 0, 0, 1])
        c = np.dot(np.identity(4), [90, -30, 105, 1])
        print('Posição em relação ao solo:', )
        a2 = self.Trans(3, -6, -110)
        a1 = self.Rot('x', -90)
        a0 = self.Rot('z', 180)
        T = np.dot(a0, a1)
        T = np.dot(T, a2)
        
        #print('T', T)
        
        AA = np.dot(T, c)
        
        print('Ponto em relação ao braço:', AA)


def main(args=None):
    rclpy.init(args=args)
    node = FaceNode()
    node.transform()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
