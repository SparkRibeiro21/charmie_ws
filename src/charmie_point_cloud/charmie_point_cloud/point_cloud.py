#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String, Float32
from charmie_interfaces.msg import NeckPosition, RequestPointCloud, RetrievePointCloud
from geometry_msgs.msg import Point, Pose2D
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import Normalize
import time


DEBUG_DRAW = True

GRAF3D = 1

x1 = -1     # coordenadas do rato
y1 = -1     # coordenadas do rato
WIDTH = 100 # largura da Bounding box
HEIGHT = 100 # altura da Bounding box

MAX_DIST = 6000
MIN_DIST = 300

linhas = 720
colunas = 1280

Z_MIN_A_VER = -1100
inc = 5

ELEV = -150
AZIM = -35
DIM = 6000

class PointCloud():
    def __init__(self):
        print("New PointCloud Class Initialised")

        global linhas, colunas
        # print(linhas, colunas)
        
        # Parametros intrinsecos da Camera (Dados pelo Tiago por email)
        self.fx = 633.811950683593  # Distancia Focal em pixels (x-direction)
        self.fy = 633.234680175781  # Distancia Focal em pixels (y-direction)
        self.cx = 629.688598632812  # Ponto Principal em pixels (x-coordinate)
        self.cy = 393.705749511718  # Ponto Principal em pixels (y-coordinate)

        # cinematica do cabeça do robo
        self.DOF = 3
        self.teta = [  0,   0,   0]
        self.alfa = [ 90, -90,   0]
        self.d =    [ 25,   0, 145]
        self.l =    [ 28,   0, 130]

        # estes 4 não deviam ter uma variavel???????????
        self.T=np.identity(4)
        self.inverse_matrix = np.zeros([4, 4]) 
        # print(self.inverse_matrix)

        self.Z_MIN_A_VER = -1100
        self.inc = 5

        self.rgb_img_pc = np.zeros((linhas,colunas,3), dtype=np.uint8)
        self.depth_img_pc = np.zeros((linhas,colunas), dtype=float)

        self.points = []  # Inicializa um array vazio para os points
        self.lista_points = []   # [u,v] [linha, coluna]  [cima-baixo, esquerda-direita]
        self.FINAL1 = []
        self.FINAL2 = []
        self.FINAL3 = []
        self.ESCALA = 8  # Por questões de eficiencia, só vamos considerar 1 pixel em cada 4

        self.retrieve_bbox = False

        if GRAF3D == 1:
            # inicializações necessárias para desenhar graficos com o MatPlotLib
            self.fig = plt.figure()
            #fig_manager = plt.get_current_fig_manager()
            #fig_manager.full_screen_toggle()
            plt.ion()  # Enable interactive mode
            self.ax1 = self.fig.add_subplot(221)
            self.ax2 = self.fig.add_subplot(222)
            self.ax3 = self.fig.add_subplot(223)
            self.ax4 = self.fig.add_subplot(224, projection='3d')
            self.ax4.mouse_init()


        self.actualiza_tetas(' ')
        self.robo()      # calcula a cinematica uma vez para os vectores terem valore
        self.converter_2D_3D(0, 0, linhas, colunas)

        if GRAF3D == 1:
            self.graficos()

    def Trans(self, tx, ty, tz):
        M = np.identity(4)
        M[0][3] = tx
        M[1][3] = ty
        M[2][3] = tz
        return M

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

    def braco(self, talfa, tteta, td, tl):
        a=self.Rot('z', tteta)
        b=self.Trans(tl, 0, td)
        c=self.Rot('x', talfa)
        temp = np.dot(a,b)
        T = np.dot(temp,c)
        return T
    
    def robo(self):
        self.T=np.identity(4)
        for i in range(self.DOF):
            self.T=np.dot(self.T,self.braco(self.alfa[i], self.teta[i], self.d[i], self.l[i]))
        self.inverse_matrix = np.linalg.inv(self.T)
        return self.inverse_matrix

    def actualiza_tetas(self, tecla):
        global ELEV, AZIM, inc, DIM, Z_MIN_A_VER, WIDTH, HEIGHT, x1, y1

        if tecla==ord('1'):
            self.teta[0] = self.teta[0] +inc
        elif tecla==ord('2'):
            self.teta[1] = self.teta[1] +inc

        elif tecla==ord('q'):
            self.teta[0] = self.teta[0] -inc
        elif tecla==ord('w'):
            self.teta[1] = self.teta[1] -inc

        elif tecla == ord('a'):
            ELEV = ELEV + inc
        elif tecla == ord('z'):
            ELEV = ELEV - inc

        elif tecla == ord('I'):
            inc = inc + 1
        elif tecla == ord('i'):
            inc = inc - 1

        elif tecla == ord('x'):
            AZIM = AZIM - inc
        elif tecla == ord('c'):
            AZIM = AZIM + inc

        elif tecla == ord('+'):
            DIM = DIM + 200
        elif tecla == ord('-'):
            DIM = DIM - 200

        elif tecla == ord('n'):
            Z_MIN_A_VER = Z_MIN_A_VER + 100
        elif tecla == ord('m'):
            Z_MIN_A_VER = Z_MIN_A_VER - 100

        elif tecla == ord('7'):
            WIDTH = WIDTH - 10
        elif tecla == ord('8'):
            WIDTH = WIDTH + 10
        elif tecla == ord('9'):
            HEIGHT = HEIGHT - 10
        elif tecla == ord('0'):
            HEIGHT = HEIGHT + 10

        elif tecla == ord('S'):
            self.ESCALA = self.ESCALA + 1
        elif tecla == ord('s'):
            self.ESCALA = self.ESCALA - 1

        elif tecla==ord('.'):
            for i in range(self.DOF):
                self.teta[i] = 0

        self.show_images()


    def show_images(self):
        # quando pressionar uma tecla qualquer, redesenha tudo para actualizar as variáveis
        # nova = self.jpg.copy()
        # cv2.rectangle(self.rgb_img_pc, (x1, y1), (x1+WIDTH, y1+HEIGHT), 255, 2)
        # cv2.imshow("New Image", self.rgb_img_pc)
        # cv2.imshow("Depth Image", self.depth_img_pc)
        # print("RGB:", self.rgb_img_pc.shape, "DEPTH:", self.depth_img_pc.shape)
        # cv2.waitKey(1)
        pass

    def especifico(self, um, vm, height, width):

        un = 1+int((height-1)/self.ESCALA)       # não é necessário nesta rotina mas pode ser noutras
        vn = 1+int((width-1)/self.ESCALA)
        ut = int((um / self.ESCALA)+0.5)
        vt = int((vm / self.ESCALA)+0.5)
        posicao = (ut-1)*vn + vt
        return self.points[posicao]
    

    
    def converter_2D_3D(self, u, v, height, width):
        
        self.points = []  # Initialize an empty list for storing points
        self.FINAL1 = []
        self.FINAL2 = []
        self.FINAL3 = []

        # print(self.inverse_matrix)
        # calcula toda a bounding box        
        # if self.retrieve_bbox: # ver no futuro se faz sentido
        for u1 in range(u, u+height, self.ESCALA):
            for v1 in range(v, v+width, self.ESCALA):
                depth = self.depth_img_pc[u1][v1]
                
                # print(u1, v1, depth)
                X = (v1 - self.cx) * depth / self.fx
                Y = (u1 - self.cy) * depth / self.fy
                Z = depth

                xn = Z
                yn = -X
                zn = -Y
                # CONFIRMAR INVERSE MATRIXX É MESMO T
                result = np.dot(self.inverse_matrix, [xn, yn, zn, 1])
                # print(".")
                #if result[2]>Z_MIN_A_VER:  # Para limpar os pontos do chão
                self.points.append([result[0], result[1], result[2]])
                self.FINAL3.append([result[0], result[1], result[2]])
        #print('FINAL3=', FINAL3)

        # calcula o ponto central da bounding box
        coord = self.especifico(height/ 2, width/2, height, width)
        self.FINAL1 = coord
        print('FINAL1=', self.FINAL1)

        # DEPOIS TEM QUE SE APAGAR ISTO
        # calcula uma lista de pontos
        # self.lista_points = []

        print("lp:", self.lista_points)

        nova = self.rgb_img_pc.copy()

        for i in range(len(self.lista_points)):
            # cv2.circle(nova, (self.lista_points[i][1], self.lista_points[i][0]), 3, (255,255,255), 1)
            coord = self.especifico(self.lista_points[i][0]-u, self.lista_points[i][1]-v, height, width)
            self.FINAL2.append(coord)
        cv2.imshow("DEBUG DRAWINGS", nova)
        print('FINAL2=', self.FINAL2)

        self.points = np.array(self.points)  # Convert the list of points to a numpy array para desenhar os graficos a 3D
    
    def graficos(self):

        global DIM

        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()

        # Y-X Plane (TOP View)
        norm = Normalize(vmin=min(self.points[:,2]), vmax=max(self.points[:,2]))
        colors = norm(self.points[:,2])  # Map z values to the [0, 1] range using the Normalize object
        #points = points[points[:, 2].argsort()[::-1]]
        self.ax1.scatter(self.points[:, 1], self.points[:, 0], marker='.', c=colors, cmap='hsv')
        self.ax1.set_xlabel('Y')
        self.ax1.set_ylabel('X')
        self.ax1.set_title('Y-X Plane - TOP view')
        #ax1.set_ylim(max(points[:, 1]), min(points[:, 1]))
        self.ax1.set_xlim( DIM,-DIM)
        self.ax1.set_ylim(0, DIM)
        self.ax1.set_aspect('equal')

        # X-Z Plane (right SIDE View)
        norm = Normalize(vmin=min(self.points[:,1]), vmax=max(self.points[:,1]))
        colors = norm(self.points[:,1])  # Map z values to the [0, 1] range using the Normalize object
        #points = points[points[:, 1].argsort()]
        self.ax2.scatter(self.points[:, 0], self.points[:, 2], marker='.', c=colors, cmap='hsv')
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Z')
        self.ax2.set_title('X-Z Plane - SIDE view')
        self.ax2.set_xlim(0, DIM)
        self.ax2.set_ylim(-DIM//2, DIM//2)
        self.ax2.set_aspect('equal')

        # Z-Y Plane (FRONT View)
        norm = Normalize(vmin=min(self.points[:,2]), vmax=max(self.points[:,2]))
        colors = norm(self.points[:,2])  # Map z values to the [0, 1] range using the Normalize object
        self.ax3.scatter(self.points[:, 1], self.points[:, 2], marker='.', c=colors, cmap='hsv')
        self.ax3.set_xlabel('Y')
        self.ax3.set_ylabel('Z')
        self.ax3.set_title('Y-Z Plane - FRONT view')
        self.ax3.set_xlim( DIM, -DIM)
        self.ax3.set_ylim(-DIM//2,  DIM//2)
        self.ax3.set_aspect('equal')

        # jet rainbow, hsv brg
        norm = Normalize(vmin=min(self.points[:,2]), vmax=max(self.points[:,2]))
        colors = norm(self.points[:,2])  # Map z values to the [0, 1] range using the Normalize object

        # 3D Orthogonal View
        self.ax4.scatter(self.points[:, 0], self.points[:, 1], self.points[:, 2], marker='.', c=colors, cmap='hsv')
        self.ax4.set_xlabel('X')
        self.ax4.set_ylabel('Y')
        self.ax4.set_zlabel('Z')
        self.ax4.set_title('Orthogonal')
        self.ax4.set_xlim(0,  DIM)
        self.ax4.set_ylim( DIM, -DIM)
        self.ax4.set_zlim( DIM//2, -DIM//2)
        #ax4.set_xlim(    0, 3000)
        #ax4.set_ylim(3000,     0)
        #ax4.set_zlim(2000, -2000)
        self.ax4.set_aspect('equal')
        self.ax4.view_init(elev=ELEV, azim=AZIM)

        plt.pause(0.1)


class PointCloudNode(Node):

    def __init__(self):
        super().__init__("PointCloud")
        self.get_logger().info("Initialised CHARMIE PointCloud Node")
        
        # Intel Realsense Subscribers
        self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_callback, 10)
        
        # Neck Position
        self.neck_get_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos", self.get_neck_position_callback, 10)

        # RequestPointCloud
        self.request_point_cloud_subscriber = self.create_subscription(RequestPointCloud, "ask_point_cloud", self.get_request_point_cloud_callback, 10)

        # RetrievePointCloud
        self.retrieve_point_cloud_publisher = self.create_publisher(RetrievePointCloud, "get_point_cloud", 10)

        # self.create_timer(0.01, self.get_request_point_cloud_callback)

        # Point Cloud Instance
        self.br = CvBridge()
        self.rgb_img = Image()
        self.depth_img = Image()
        self.pcloud = PointCloud()
        self.neck_position = NeckPosition()

        self.tempinho = time.perf_counter()

    def get_color_image_callback(self, img: Image):
        self.rgb_img = img
        # print("Received RGB Image")

    def get_aligned_depth_image_callback(self, img: Image):
        self.depth_img = img
        # print("Received Depth Image")

    def get_neck_position_callback(self, neck_pos: NeckPosition):
        self.neck_position = neck_pos
        self.pcloud.teta[0] = 180 - neck_pos.pan
        self.pcloud.teta[1] = 190 - neck_pos.tilt ###### ALTERAR PARA 180
        # print("Received Neck Position: (", neck_pos.pan, ",", neck_pos.tilt, ") - (", self.pcloud.teta[1], ",", self.pcloud.teta[2], ")")

    def get_request_point_cloud_callback(self, req: RequestPointCloud): #, req: RequestPointCloud):
        
        if self.depth_img.height > 0 and self.rgb_img.height > 0: # prevents doing this code before receiving images

            rgb_frame = self.br.imgmsg_to_cv2(self.rgb_img, "bgr8")
            depth_frame = self.br.imgmsg_to_cv2(self.depth_img, desired_encoding="passthrough")
            
            width = self.rgb_img.width
            height = self.rgb_img.height

            depth_frame_res = cv2.resize(depth_frame, (width, height), interpolation = cv2.INTER_NEAREST)

            depth_frame_res[depth_frame_res > 6000] = 0
            depth_frame_res[depth_frame_res <  300] = 0

            self.pcloud.rgb_img_pc = rgb_frame
            self.pcloud.depth_img_pc = depth_frame_res
        
            self.pcloud.lista_points = []
            for i in range(len(req.requested_point_coords)):
                # p = Pose2D()
                # p = req.
                self.pcloud.lista_points.append([req.requested_point_coords[i].y, req.requested_point_coords[i].x])   # [u,v] [linha, coluna]  [cima-baixo, esquerda-direita]

            print(self.pcloud.lista_points)
            self.pcloud.retrieve_bbox = req.retrieve_bbox 

            tecla = cv2.waitKey(1)  # le uma tecla
            if tecla != 27:         # se não for a tecla de ESCAPE, faz todo o processamento
                self.pcloud.actualiza_tetas(tecla)  # actualiza as variaveis Theta da cinematica
            else:
                self.pcloud.show_images()
                
            T = self.pcloud.robo()              # calcula a matris inversa da cabeça toda

            # imprime as variaveis todas apenas para DEBUG no terminal
            print("[ ", end=' ')
            for i in self.pcloud.teta[0:2]:
                print("{:>3.0f}".format(i), end=' ')
            print("]  -> ", end=' ')
            '''
            for i in T[0:3, 3]:
                print("{:>8.3f}".format(i), end=' ')
            '''
            print('inc=', inc, '  ELEV=', ELEV, '  AZIM=', AZIM, ' DIM=', DIM, ' Z_MIN_A_VER=', Z_MIN_A_VER, ' WIDTH=', WIDTH, ' HEIGHT=', HEIGHT, ' ESCALA=', self.pcloud.ESCALA)
            
            print('tempo_frame = ', time.perf_counter() - self.tempinho)   # imprime o tempo de calculo em segundos
            
            self.tempinho = time.perf_counter()
            # self.pcloud.converter_2D_3D(y1, x1, HEIGHT, WIDTH)      # converte as coordenadas de (u,v) para 3D (x, y, z)
            # self.pcloud.converter_2D_3D(0, 0, height, width)      # converte as coordenadas de (u,v) para 3D (x, y, z)
            self.pcloud.converter_2D_3D(req.box_top_left_y, req.box_top_left_x, req.box_height, req.box_width)      # converte as coordenadas de (u,v) para 3D (x, y, z)
            print('tempo_conver = ', time.perf_counter() - self.tempinho)   # imprime o tempo de calculo em segundos
            self.tempinho = time.perf_counter()
            if GRAF3D == 1:
                self.pcloud.graficos()      # mostra os graficos



            # geometry_msgs/Point center_coords
            # geometry_msgs/Point[] requested_point_coords
            # geometry_msgs/Point[] bbox_point_coords

            answer = RetrievePointCloud()
            answer.center_coords.x = self.pcloud.FINAL1[0]
            answer.center_coords.y = self.pcloud.FINAL1[1]
            answer.center_coords.z = self.pcloud.FINAL1[2]
            print(answer.center_coords)
            print("----")

            for i in range(len(self.pcloud.FINAL2)):
                p = Point()
                p.x = self.pcloud.FINAL2[i][0]
                p.y = self.pcloud.FINAL2[i][1]
                p.z = self.pcloud.FINAL2[i][2]
                print(p)
                answer.requested_point_coords.append(p)
            print("====")

            if req.retrieve_bbox:
                for i in range(len(self.pcloud.FINAL3)):
                    p = Point()
                    p.x = self.pcloud.FINAL3[i][0]
                    p.y = self.pcloud.FINAL3[i][1]
                    p.z = self.pcloud.FINAL3[i][2]
                    answer.bbox_point_coords.append(p)

                    print(p)
                print("####")
            self.retrieve_point_cloud_publisher.publish(answer)

            




def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    rclpy.spin(node)
    rclpy.shutdown()