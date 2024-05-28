#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from charmie_interfaces.msg import NeckPosition, PointCloudCoordinates
from charmie_interfaces.srv import GetPointCloud
from geometry_msgs.msg import Point, Pose2D
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import math
import time

# x increases from the back of the robot to the front of the robot
# y increases from the robot left to the robot right
# z increases from the floor to the ceiling

# maximum and minimum distnace considered, outside this range is 0
MAX_DIST = 500
MIN_DIST = 70

Z_MIN_OBSTACLES = 200 # when applying the point cloud to the obstacles, this is the minimum height to be considered an object, filter the floor
Z_MAX_OBSTACLES = 1900 # when applying the point cloud to the obstacles, this is the minimum height to be considered an object, filter the ceiling

# shifts from the center of the bottom servo to the center of the robot platform
X_SHIFT = 50
Z_SHIFT = 1260

class PointCloud():
    def __init__(self):
        # print("New PointCloud Class Initialised")

        self.linhas = 720
        self.colunas = 1280
        # print(linhas, colunas)
        
        # Parametros intrinsecos da Camera (Dados pelo Tiago)
        self.fx = 658.65612382  # Distancia Focal em pixels (x-direction)
        self.fy = 658.5626897  # Distancia Focal em pixels (y-direction)
        self.cx = 642.88868778  # Ponto Principal em pixels (x-coordinate)
        self.cy = 346.93829812  # Ponto Principal em pixels (y-coordinate)

        self.teta = [  0,   0,   0] # neck values to adjust the kinematics

        self.rgb_img_pc = np.zeros((self.linhas, self.colunas,3), dtype=np.uint8)
        self.depth_img_pc = np.zeros((self.linhas, self.colunas), dtype=np.uint8)

        self.ESCALA = 16  # Por questões de eficiencia, só vamos considerar 1 pixel em cada 4<  QA

        self.RECEBO = []
        self.ENVIO = []

        self.T = np.identity(4)
        self.robo()

            
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


    def robo(self):
        A4 = self.Trans(90, 11.5, 195)
        A3 = self.Rot('y', self.teta[1])
        A2 = self.Trans(30, 0, 25)
        A1 = self.Rot('z', self.teta[0])
        T = np.dot(A1, A2)
        T = np.dot(T, A3)
        T = np.dot(T, A4)
        self.T = T
        

    def converter_2D_3D_unico(self, u, v):
        
        # this is to prevent 'bug' regarding maximum values on each axis
        if u >= self.linhas:
            u = self.linhas-1
        
        if v >= self.colunas:
            v = self.colunas-1

        # print(u, v)

        depth = self.depth_img_pc[u][v]
        if depth == 0:      # Só faz os cálculos de o ponto for válido (OPTIMIZAÇÃO)

            # casos especiais, mas só em coordenadas especificas (não faz para a bounding box toda)
            # procuro a distancia mais próxima nos pixeis vizinhos
            raio = 1
            while np.all(self.depth_img_pc[u - raio:u + raio + 1, v - raio:v + raio + 1] == 0):
                raio += 1
            nao_zeros = (self.depth_img_pc[u - raio:u + raio + 1, v - raio:v + raio + 1] != 0)
            depth = np.min(self.depth_img_pc[u - raio:u + raio + 1, v - raio:v + raio + 1][nao_zeros])
            #print('u, v, min ', u, v, depth)

        Z = depth
        X = (v - self.cx) * depth / self.fx
        Y = (u - self.cy) * depth / self.fy

        xn = Z
        yn = -X
        zn = -Y
        result = np.dot(self.T, [xn, yn, zn, 1])
        result[0] += X_SHIFT
        result[2] += Z_SHIFT  # Z=0 is the floor

        result = result[0:3].astype(np.int16)
        result = result.tolist()
        return result

    
    def converter_2D_3D(self, u, v, height, width):
        
        points = []

        # calcula toda a bounding box        
        for u1 in range(u, u+height, self.ESCALA):
            for v1 in range(v, v+width, self.ESCALA):
                depth = self.depth_img_pc[u1][v1]
                if depth == 0:      # Só faz os cálculos se o ponto for válido (OPTIMIZAÇÃO)
                    result = np.dot(np.identity(4), [0, 0, 0, 1]) 
                else:
                    X = (v1 - self.cx) * depth / self.fx
                    Y = (u1 - self.cy) * depth / self.fy
                    Z = depth

                    xn = Z
                    yn = -X
                    zn = -Y

                    result = np.dot(self.T, [xn, yn, zn, 1])
                    result[0] += X_SHIFT
                    result[2] += Z_SHIFT  # Z=0 is the floor

                    result = result[0:3].astype(np.int16)
                    result = result.tolist()

                points.append(result)
        return points
    

class PointCloudNode(Node):

    def __init__(self):
        super().__init__("PointCloud")
        self.get_logger().info("Initialised CHARMIE PointCloud Node")
        
        # Intel Realsense Subscribers
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_callback, 10)
        
        # Neck Position
        self.neck_get_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos_topic", self.get_neck_position_callback, 10)

        # SERVICES:
        # Main receive commads 
        self.server_point_cloud = self.create_service(GetPointCloud, "get_point_cloud_head", self.callback_point_cloud) 
        self.get_logger().info("Point Cloud Server has been started")

        # Point Cloud Instance
        self.br = CvBridge()
        self.rgb_img = Image()
        self.depth_img = Image()
        self.pcloud = PointCloud()

        self.tempo_calculo = 0
        self.tempo_frame = 0

    def get_color_image_head_callback(self, img: Image):
        self.rgb_img = img
        # print("Received RGB Image")

    def get_aligned_depth_image_callback(self, img: Image):
        self.depth_img = img
        # print("Received Depth Image")

    def get_neck_position_callback(self, neck_pos: NeckPosition):
        # change the axis to fit the kinematics
        self.pcloud.teta[0] = neck_pos.pan
        self.pcloud.teta[1] = -neck_pos.tilt
        # print("Received Neck Position: (", neck_pos.pan, ",", neck_pos.tilt, ") - (", self.pcloud.teta[0], ",", self.pcloud.teta[1], ")")

    def callback_point_cloud(self, request, response):

        # print(request)

        # Type of service received:
        # BoundingBoxAndPoints[] data # bounding box and specific points inside the bounding box  
        # bool retrieve_bbox # if it is intended to get the full bounding box of 3D points returned, saves data transitions 
        # ---
        # PointCloudCoordinates[] coords # returns the selected 3D points (the bounding box center, the custom ones and the full bounding box)
        
        if self.depth_img.height > 0 and self.rgb_img.height > 0: # prevents doing this code before receiving images

            rgb_frame = self.br.imgmsg_to_cv2(self.rgb_img, "bgr8")
            depth_frame = self.br.imgmsg_to_cv2(self.depth_img, desired_encoding="passthrough")
            
            width = self.rgb_img.width
            height = self.rgb_img.height

            depth_frame_res = cv2.resize(depth_frame, (width, height), interpolation = cv2.INTER_NEAREST)

            depth_frame_res[depth_frame_res > MAX_DIST] = 0
            depth_frame_res[depth_frame_res < MIN_DIST] = 0

            self.pcloud.rgb_img_pc = rgb_frame
            self.pcloud.depth_img_pc = depth_frame_res
        
            self.pcloud.RECEBO = []
            for i in range(len(request.data)):
                aux = []
                aux.append([request.data[i].bbox.box_top_left_y, request.data[i].bbox.box_top_left_x, request.data[i].bbox.box_height, request.data[i].bbox.box_width])

                a = []
                for j in range(len(request.data[i].requested_point_coords)):
                    a.append([int(request.data[i].requested_point_coords[j].y), int(request.data[i].requested_point_coords[j].x)])

                aux.append(a)
                self.pcloud.RECEBO.append(aux)

            self.pcloud.ENVIO = []      # limpo a variavel onde vou guardar as minhas respostas, para este novo ciclo
            self.tempo_calculo = time.perf_counter()

            # calculo dos vários Bounding Boxes
            self.pcloud.robo() 
            for bbox in self.pcloud.RECEBO:

                # le os dados da BouundingBox
                # u_inicial, v_inicial, HEIGHT, WIDTH = bbox[0]
                u_inicial, v_inicial, bb_height, bb_width = bbox[0]

                # print("CENTER")
                # calcula o ponto do centro
                # this is still a change in progess. Rather than returning the x,y,z of the center point of the bounding box, 
                # we are trying to compute the mean of the points that interest us, since the center point of the bounding box may
                # be in the object/person we are tryng to get the position
                # old version - using the center of the bounding box:
                # resp_centro = self.pcloud.converter_2D_3D_unico(u_inicial + HEIGHT//2, v_inicial + WIDTH//2)
                # new version:

                resp_todos = []
                resp_todos = self.pcloud.converter_2D_3D(u_inicial, v_inicial, bb_height, bb_width)
                uteis = [row for row in resp_todos if (row[0]!=0 or row[1]!=0 or row[2]!=0)] # limpa os elementos [0, 0, 0]
                # print(uteis, len(uteis))
                
                if len(uteis) == 0:
                    resp_centro = self.pcloud.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
                else:
                    x_coord = np.array(uteis)[:, 0]                             # Extrai a coordenada X
                    y_coord = np.array(uteis)[:, 1]  # Extrai a coordenada X
                    z_coord = np.array(uteis)[:, 2]  # Extrai a coordenada X
                    x_min, x_max, _, _ = cv2.minMaxLoc(x_coord)
                    y_min, y_max, _, _ = cv2.minMaxLoc(y_coord)
                    z_min, z_max, _, _ = cv2.minMaxLoc(z_coord)
                    y_min = y_min + (y_max-y_min)*0.05
                    y_max = y_max - (y_max-y_min)*0.05
                    z_min = z_min + (z_max-z_min)*0.05
                    z_max = z_max - (z_max-z_min)*0.05
                    uteis = [row for row in uteis if ((row[1]>y_min and row[1]<y_max) and (row[2]>z_min and row[2]<z_max))] # limpa os elementos [0, 0, 0]
                    if len(uteis) == 0:
                        resp_centro = self.pcloud.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
                    else:
                        bin_edges = np.arange(min(x_coord), max(x_coord) + 100, 100)    # Cria a lista de limites para o histograma
                        hist, bin_edges = np.histogram(x_coord, bins=bin_edges)     # Usa np.histogram para contar as ocorrencias
                        _, maior, _, posicao = cv2.minMaxLoc(hist)                  # calcula o máximo
                        if posicao[1]>0:
                            Xmin = bin_edges[posicao[1] - 1]  # calcula a coordenada X ao objeto
                        else:
                            Xmin = bin_edges[posicao[1]    ]  # calcula a coordenada X ao objeto
                        # print('len posicao', len(bin_edges))
                        if posicao[1]<len(bin_edges)-2:
                            Xmax = bin_edges[posicao[1] + 2]
                        else:
                            Xmax = bin_edges[posicao[1] + 1]
                        uteis_uteis = [row for row in uteis if (row[0]>=Xmin and row[0]<=Xmax)] # filtro apenas os elementos no pico do histograma
                        # for i in uteis_uteis:
                            # print(i[0], '\t', i[1], '\t', i[2])
                        centroid = np.mean(uteis_uteis, axis=0)

                        if np.isnan(centroid).any():
                            resp_centro = self.pcloud.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
                        else:
                            resp_centro = centroid
                                    
                        # print("centroid:", centroid)
                        # self.get_logger().info(f"Centroid: {centroid}")

                # print("REQUESTED")
                # calcula a lista de pontos
                resp_outros = []
                for i in bbox[1]:
                    temp = self.pcloud.converter_2D_3D_unico(i[0], i[1])
                    resp_outros.append(temp)

                # Guarda todas as respostas na variavel ENVIO
                temp = []
                temp.append(resp_centro)
                temp.append(resp_outros)
                temp.append(resp_todos)
                self.pcloud.ENVIO.append(temp)

            # convert ENVIO into RetrievePointCloud ROS Variable
            ret = []
            if len(self.pcloud.ENVIO) > 0:
                for cc in self.pcloud.ENVIO:
                    # print(cc)

                    pcc = PointCloudCoordinates()

                    point_c = Point()
                    point_c.x = float(cc[0][0])
                    point_c.y = float(cc[0][1])
                    point_c.z = float(cc[0][2])

                    pcc.center_coords = point_c

                    kp_list = []
                    for kp in cc[1]:
                        # print("kp:", kp)
                        point_kp = Point()
                        point_kp.x = float(kp[0])
                        point_kp.y = float(kp[1])
                        point_kp.z = float(kp[2])
                        kp_list.append(point_kp)

                    pcc.requested_point_coords = kp_list

                    bb_list = []
                    if request.retrieve_bbox:
                        for bb in cc[2]:
                            # print("bb:", bb)
                            point_bb = Point()
                            point_bb.x = float(bb[0])
                            point_bb.y = float(bb[1])
                            point_bb.z = float(bb[2])
                            bb_list.append(point_bb)

                    pcc.bbox_point_coords = bb_list

                    ret.append(pcc)

            response.coords = ret
            
            # imprime os tempos de processamento e da frame
            self.get_logger().info(f"Point Cloud Time: {time.perf_counter() - self.tempo_calculo}")
            # print('tempo calculo = ', time.perf_counter() - self.tempo_calculo)   # imprime o tempo de calculo em segundos
            # print('tempo frame = ', time.perf_counter() - self.tempo_frame)   # imprime o tempo de calculo em segundos
            # self.tempo_frame = time.perf_counter()

        # print(response)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    rclpy.spin(node)
    rclpy.shutdown()