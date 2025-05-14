#!/usr/bin/env python3
from geometry_msgs.msg import Point
import numpy as np
import time
import cv2

class Camera():
    def __init__(self, camera, fx, fy, cx, cy, width, height, max_dist, min_dist):
        
        self.camera = camera        # name da camara
        self.fx = fx                # Distancia Focal em pixels (x-direction)
        self.fy = fy                # Distancia Focal em pixels (y-direction)
        self.cx = cx                # Ponto Principal em pixels (x-coordinate)
        self.cy = cy                # Ponto Principal em pixels (y-coordinate)
        self.width = width          # Largura da Imagem
        self.height = height        # Altura da Imagem
        self.max_dist = max_dist    # Maxima distancia detetada pela camara
        self.min_dist = min_dist    # Minima distancia detetada pela camara

### CAMERAS INTRINSIC PARAMETERS ###
# Read from /camera_info topic of each camera (colour and depth_aligned)
# The used intrinsic parameters are from the k matrix:

# [fx 0  cx]
# [0  fy cy]
# [0  0  1 ]

# k:
# - fx
# - 0
# - cx
# - 0
# - fy
# - cy
# - 0
# - 0
# - 1

# fx: Distancia Focal em pixels (x-direction)
# fy: Distancia Focal em pixels (y-direction)
# cx: Ponto Principal em pixels (x-coordinate)
# cy: Ponto Principal em pixels (y-coordinate)


### The FR axis system does not match ROS2 standards, must be converted
# FR    (x:right, y:down, z:front)
# ROS2  (x:front, y:left, z:up)
# FR to ROS2 Conversion:
# ROS2 = FR
# X    =  z
# Y    = -x
# Z    = -y 


class PointCloud():
    def __init__(self):

        self.head_camera = Camera(camera="head", fx=420.814270019531, fy=420.430999755859, cx=417.168701171875, cy=262.330047607422, width=848, height=480, max_dist=6000, min_dist=300)
        self.hand_camera = Camera(camera="hand", fx=434.083312988281, fy=433.659149169922, cx=421.407775878906, cy=236.390808105469, width=848, height=480, max_dist=1000, min_dist=70 )
        self.base_camera = Camera(camera="base", fx=545.646362304688, fy=545.646362304688, cx=321.721069335937, cy=238.693023681641, width=640, height=480, max_dist=6000, min_dist=400)
    
    # Receives a (u, v) point and a depth image of the corresponding camera, returns the (x,y,z) according to the camera TF (filtered in case depth point can not be retrieved)
    def convert_pixel_to_3dpoint(self, depth_img, camera, pixel):

        match camera:
            case "head":
                camera_used = self.head_camera
            case "hand":
                camera_used = self.hand_camera
            case "base":
                camera_used = self.base_camera

        u = pixel[0]
        v = pixel[1]
        
        # this is to prevent 'bug' regarding maximum values on each axis
        if u >= camera_used.height:
            u = camera_used.height-1
        
        if v >= camera_used.width:
            v = camera_used.width-1

        # print(u, v)

        depth = depth_img[u][v]
        if depth == 0: # Only makes calculations if depth is invalid

            # new version (post fixed bug: inspection robocup 2024)
            raio = 1
            max_radius = 16
            while np.all(depth_img[u - raio:u + raio + 1, v - raio:v + raio + 1] == 0) and raio <= max_radius:
                raio += 1
            
            if raio < max_radius:
                nao_zeros = (depth_img[u - raio:u + raio + 1, v - raio:v + raio + 1] != 0)
                depth = np.min(depth_img[u - raio:u + raio + 1, v - raio:v + raio + 1][nao_zeros])
                # print('u, v, min ', u, v, depth)
            else: # error case
                depth = camera_used.min_dist
                depth = 0
                # print("ERROR - BUG INSPECTION ROBOCUP 2024")

        point3d = Point()
        point3d.x = float(depth/1000)
        point3d.y = -float(((v - camera_used.cx) * depth / camera_used.fx)/1000)
        point3d.z = -float(((u - camera_used.cy) * depth / camera_used.fy)/1000)

        return point3d

    # Receives a segmentation mask and a depth image of the corresponding camera, returns the (x,y,z) according to the camera TF (0,0,0 if no depth point is available inside mask)
    def convert_mask_to_3dpoint(self, depth_img, camera, mask):

        # aaa = time.time()

        match camera:
            case "head":
                camera_used = self.head_camera
            case "hand":
                camera_used = self.hand_camera
            case "base":
                camera_used = self.base_camera
        
        b_mask = np.zeros(depth_img.shape[:2], np.uint8) # creates new empty window
        contour = mask
        contour = contour.astype(np.int32)
        contour = contour.reshape(-1, 1, 2)
        cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED) # creates mask window with just the inside pixesl of the detected objects
        depth_frame_res_mask = depth_img.copy()
        depth_frame_res_mask[b_mask == 0] = [0]
        # cv2.imwrite('output_image.png', depth_frame_res_mask)

        non_zero_indices = np.nonzero(depth_frame_res_mask)
        non_zero_values = depth_frame_res_mask[non_zero_indices] 
        point3d = Point()

        if non_zero_values.size:

            # depth_average = np.mean(non_zero_values)
            depth_median = np.median(non_zero_values)
            depth = depth_median
            u = np.mean(non_zero_indices[0])
            v = np.mean(non_zero_indices[1])
            # print("avg np:", depth, u, v)

            point3d.x = float(depth/1000)
            point3d.y = -float(((v - camera_used.cx) * depth / camera_used.fx)/1000)
            point3d.z = -float(((u - camera_used.cy) * depth / camera_used.fy)/1000)

        # else: # sends the point as x:0, y:0, z:0 
            # there are no elements on the non_zero_indices from the mask

        # print("PC TIME:", time.time() - aaa)

        return point3d
    
    def convert_bbox_to_3d_point(self, depth_img, camera, bbox):

        u_inicial = int(bbox.xyxy[0][1])
        v_inicial = int(bbox.xyxy[0][0])
        bb_height = int(bbox.xyxy[0][3]) - int(bbox.xyxy[0][1])
        bb_width = int(bbox.xyxy[0][2]) - int(bbox.xyxy[0][0])

        mask = np.array([
            [v_inicial, u_inicial],
            [v_inicial+bb_width, u_inicial],
            [v_inicial+bb_width, u_inicial+bb_height],
            [v_inicial, u_inicial+bb_height],
            [v_inicial, u_inicial]
        ])

        # converts the bounding box into a mask and does the same calculations for a segmentation mask
        # The goal is to have everything working with segmentation masks and not bounding boxes.
        # But we leave this here in case it is necessary
        point3d = self.convert_mask_to_3dpoint(depth_img, camera, mask)

        return point3d
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    ### Receives a bounding box and a depth image of the corresponding camera, returns the (x,y,z) of the average of the filtered bounding box points according to the camera TF
    # def converter_2D_3D(self, u, v, height, width):
    def old_convert_bbox_to_3d_point(self, depth_img, camera, bbox):

        # aaa = time.time()

        match camera:
            case "head":
                camera_used = self.head_camera
            case "hand":
                camera_used = self.hand_camera
            case "base":
                camera_used = self.base_camera

        # confidence = float(bbox.conf)
        # box_top_left_x = int(bbox.xyxy[0][0])
        # box_top_left_y = int(bbox.xyxy[0][1])
        # box_width = int(bbox.xyxy[0][2]) - int(bbox.xyxy[0][0])
        # box_height = int(bbox.xyxy[0][3]) - int(bbox.xyxy[0][1])

        # print("POINT CLOUD CLASS")
        # print(bbox)







        ### UPDATED FUNCTION ALGORITHM (AFTER PointCloudNodeToClass update)

            # Receive bounding box
            # Receive depth_image
            # Get Correct Camera

            # PRE PROCESSING (MADE IN OLD PC MAIN):
            # None
 
        # Calculate the filtered version of bounding box (same code as before)
        # Clean all reference axis changes (now all values returned from PC are in reference to the camera)

        # POST PROCESSING (MADE IN OLD PC MAIN):
        # Coords must be in meters

        u_inicial = int(bbox.xyxy[0][1])
        v_inicial = int(bbox.xyxy[0][0])
        bb_height = int(bbox.xyxy[0][3]) - int(bbox.xyxy[0][1])
        bb_width = int(bbox.xyxy[0][2]) - int(bbox.xyxy[0][0])

        uteis = []
        ESCALA = 16

        x_coord = []
        y_coord = []
        z_coord = []

        # calcula toda a bounding box        
        for u1 in range(u_inicial, u_inicial+bb_height, ESCALA):
            for v1 in range(v_inicial, v_inicial+bb_width, ESCALA):
                depth = depth_img[u1][v1]
                p3d = Point()
                if depth != 0:
                    p3d.x = float(depth/1000)
                    p3d.y = -float(((v1 - camera_used.cx) * depth / camera_used.fx)/1000)
                    p3d.z = -float(((u1 - camera_used.cy) * depth / camera_used.fy)/1000)
                
                    if p3d.x!=0 or p3d.y!=0 or p3d.z!=0:
                        uteis.append(p3d)

                        x_coord.append(p3d.x)
                        y_coord.append(p3d.y)
                        z_coord.append(p3d.z)

        x_coord = np.array(x_coord)
        y_coord = np.array(y_coord)
        z_coord = np.array(z_coord)
    
        # uteis = [row for row in resp_todos if (row[0]!=0 or row[1]!=0 or row[2]!=0)] # limpa os elementos [0, 0, 0]
        # uteis = [p for p in resp_todos if (p.x!=0 or p.y!=0 or p.z!=0)] # limpa os elementos [0, 0, 0]
        # print(uteis, len(uteis))
        # print(len(uteis))
        point3d = Point()

        if len(uteis) != 0:
            pass
            #     resp_centro = self.convert_pixel_to_3dpoint()
            #     resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
            
            # x_coord = np.array(uteis)[:, 0]  # Extrai a coordenadax_coord X
            # y_coord = np.array(uteis)[:, 1]  # Extrai a coordenada X
            # z_coord = np.array(uteis)[:, 2]  # Extrai a coordenada X
            x_min, x_max, _, _ = cv2.minMaxLoc(x_coord)
            y_min, y_max, _, _ = cv2.minMaxLoc(y_coord)
            z_min, z_max, _, _ = cv2.minMaxLoc(z_coord)
            # x_min = x_min + (x_max-x_min)*0.05
            # x_max = x_max - (x_max-x_min)*0.05
            y_min = y_min + (y_max-y_min)*0.05
            y_max = y_max - (y_max-y_min)*0.05
            z_min = z_min + (z_max-z_min)*0.05
            z_max = z_max - (z_max-z_min)*0.05

            # print(x_min, x_max, y_min, y_max, z_min, z_max)

            uteis_ = [p for p in uteis if ((p.y>y_min and p.y<y_max) and (p.z>z_min and p.z<z_max))] # limpa os valores dos extremas dos 3 eixos
            # print("LEN2:", len(uteis_))
    
            # uteis = [row for row in uteis if ((row[1]>y_min and row[1]<y_max) and (row[2]>z_min and row[2]<z_max))] # limpa os valores dos extremas dos 3 eixos
            if len(uteis_) == 0:
                pass
                # resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
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
                uteis_uteis = [p for p in uteis if (p.x>=Xmin and p.x<=Xmax)] # filtro apenas os elementos no pico do histograma
                coords = np.array([[p.x, p.y, p.z] for p in uteis_uteis])
                avg_x, avg_y, avg_z = np.mean(coords, axis=0)

                # for i in uteis_uteis:
                    # print(i[0], '\t', i[1], '\t', i[2])
                # centroid = np.mean(uteis_uteis, axis=0)

                if np.isnan(coords).any():
                    pass
                    # resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
                else:
                    # resp_centro = centroid

                    point3d.x = avg_x
                    point3d.y = avg_y
                    point3d.z = avg_z
                    
                            
                # print("centroid:", centroid)
                # self.get_logger().info(f"Centroid: {centroid}")
            

        else:
            pass



        """
            # le os dados da BouundingBox
            # u_inicial, v_inicial, HEIGHT, WIDTH = bbox[0]
            u_inicial, v_inicial, bb_height, bb_width = bbox[0]

            # print("CENTER")
            # calcula o ponto do centro
            # this is still a change in progess. Rather than returning the x,y,z of the center point of the bounding box, 
            # we are trying to compute the mean of the points that interest us, since the center point of the bounding box may
            # be in the object/person we are tryng to get the position
            # old version - using the center of the bounding box:
            # resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + HEIGHT//2, v_inicial + WIDTH//2)
            # new version:

            resp_todos = []
            resp_todos = self.pcloud_head.converter_2D_3D(u_inicial, v_inicial, bb_height, bb_width)
            uteis = [row for row in resp_todos if (row[0]!=0 or row[1]!=0 or row[2]!=0)] # limpa os elementos [0, 0, 0]
            # print(uteis, len(uteis))
        
        if len(uteis) == 0:
            resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
        else:
            x_coord = np.array(uteis)[:, 0]  # Extrai a coordenada X
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
                resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
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
                    resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
                else:
                    resp_centro = centroid
                            
                # print("centroid:", centroid)
                # self.get_logger().info(f"Centroid: {centroid}")
        """
 
        # return (x,y,z) Point (in reference to camera)

        
        """
        points = []
        ESCALA = 16

        # calcula toda a bounding box        
        for u1 in range(u, u+height, ESCALA):
            for v1 in range(v, v+width, ESCALA):
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
                    result[0] += self.X_SHIFT
                    result[1] += self.Y_SHIFT
                    result[2] += self.Z_SHIFT  # Z=0 is the floor

                    result = result[0:3].astype(np.int16)
                    result = result.tolist()

                points.append(result)

        return points
    
        """




        ########## FALTA ADICIONAR AQUI O FILTRO QUE FIZEMOS (TR + FR)

        # point3d.x = 2.0
        # point3d.y = 0.0
        # point3d.z = 1.8

        return point3d
    



"""
class PointCloudNode(Node):

    def __init__(self):
        super().__init__("PointCloud")
        self.get_logger().info("Initialised CHARMIE PointCloud Node")
        
        # Intel Realsense Subscribers 
        # Head
        self.color_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/color/image_raw", self.get_color_image_head_callback, 10)
        self.aligned_depth_image_head_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_head_callback, 10)
        # Hand
        self.color_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/color/image_rect_raw", self.get_color_image_hand_callback, 10)
        self.aligned_depth_image_hand_subscriber = self.create_subscription(Image, "/CHARMIE/D405_hand/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_hand_callback, 10)        
        # Neck Position
        self.neck_get_position_subscriber = self.create_subscription(NeckPosition, "get_neck_pos_topic", self.get_neck_position_callback, 10)
        # Obstacles Head Camera Depth
        self.obstacles_head_depth_publisher = self.create_publisher(PointCloudCoordinates, "obstacles_head_depth", 10)
        
        # SERVICES:
        # Main receive commads 
        self.server_point_cloud_bb = self.create_service(GetPointCloudBB, "get_point_cloud_bb", self.callback_point_cloud_bb)  
        self.server_point_cloud_mask = self.create_service(GetPointCloudMask, "get_point_cloud_mask", self.callback_point_cloud_mask)  
        # Activate Obstacles Head Camera Depth
        self.server_activate_obstacles = self.create_service(ActivateObstacles, "activate_obstacles_head_depth", self.callback_activate_head_depth_obstacles) 
        
        self.get_logger().info("Point Cloud Servers have been started")

        # Point Cloud Instance
        self.br = CvBridge()
        self.head_rgb_img = Image()
        self.head_depth_img = Image()
        self.hand_rgb_img = Image()
        self.hand_depth_img = Image()

        self.pcloud_head = PointCloud(camera="head")
        self.pcloud_hand = PointCloud(camera="hand")    
        
        self.tempo_calculo = 0
        self.tempo_frame = 0

        self.ACTIVATE_HEAD_DEPTH_OBSTACLES = False


    def get_color_image_head_callback(self, img: Image):
        self.head_rgb_img = img
        # print("Received Head RGB Image")

    def get_aligned_depth_image_head_callback(self, img: Image):
        self.head_depth_img = img
        if self.ACTIVATE_HEAD_DEPTH_OBSTACLES:
            self.publish_head_depth_obstacles()
        # print("Received Head Depth Image")


    def get_color_image_hand_callback(self, img: Image):
        self.hand_rgb_img = img
        # print("Received Hand RGB Image")

    def get_aligned_depth_image_hand_callback(self, img: Image):
        self.hand_depth_img = img
        # print("Received Hand Depth Image")


    def get_neck_position_callback(self, neck_pos: NeckPosition):
        # change the axis to fit the kinematics
        self.pcloud_head.teta[0] = neck_pos.pan
        self.pcloud_head.teta[1] = -neck_pos.tilt
        # print("Received Neck Position: (", neck_pos.pan, ",", neck_pos.tilt, ") - (", self.pcloud_head.teta[0], ",", self.pcloud_head.teta[1], ")")

    def callback_activate_head_depth_obstacles(self, request, response):
        # print(request)

        # Type of service received:
        # bool activate_lidar_up     # activate lidar from robot body
        # bool activate_lidar_bottom # activate lidar to see floor objects
        # bool activate_camera_head  # activate head camera for 3D obstacles  
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.
        self.get_logger().info("Received Activate Obstacles %s" %("("+str(request.activate_camera_head)+")"))

        # self.robot.ACTIVATE_LIDAR_UP = request.activate_lidar_up
        # self.robot.ACTIVATE_LIDAR_BOTTOM = request.activate_lidar_bottom
        self.ACTIVATE_HEAD_DEPTH_OBSTACLES = request.activate_camera_head
        
        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Activated with selected parameters"
        return response
    
    def publish_head_depth_obstacles(self):
        # print("INSIDE DEPTH HEAD OBSTACLES FUNCTION")

        # using the same nomenclature as the service requests
        request = GetPointCloudBB.Request()
        response = GetPointCloudBB.Response()
        
        request.retrieve_bbox = True

        bb = BoundingBox()
        bb.box_top_left_x = 0
        bb.box_top_left_y = 0
        bb.box_width = 1280
        bb.box_height = 720

        get_pc = BoundingBoxAndPoints()
        get_pc.bbox = bb

        request.data = [get_pc]


        if self.head_depth_img.height > 0 and self.head_rgb_img.height > 0: # prevents doing this code before receiving images

            # rgb_frame = self.br.imgmsg_to_cv2(self.head_rgb_img, "bgr8")
            depth_frame = self.br.imgmsg_to_cv2(self.head_depth_img, desired_encoding="passthrough")
            
            width = self.head_rgb_img.width
            height = self.head_rgb_img.height

            depth_frame_res = cv2.resize(depth_frame, (width, height), interpolation = cv2.INTER_NEAREST)

            depth_frame_res[depth_frame_res > self.pcloud_head.MAX_DIST] = 0
            depth_frame_res[depth_frame_res < self.pcloud_head.MIN_DIST] = 0

            # self.pcloud_head.rgb_img_pc = rgb_frame
            self.pcloud_head.depth_img_pc = depth_frame_res
        
            self.pcloud_head.RECEBO = []
            for i in range(len(request.data)):
                aux = []
                aux.append([request.data[i].bbox.box_top_left_y, request.data[i].bbox.box_top_left_x, request.data[i].bbox.box_height, request.data[i].bbox.box_width])

                a = []
                for j in range(len(request.data[i].requested_point_coords)):
                    a.append([int(request.data[i].requested_point_coords[j].y), int(request.data[i].requested_point_coords[j].x)])

                aux.append(a)
                self.pcloud_head.RECEBO.append(aux)

            self.pcloud_head.ENVIO = []      # limpo a variavel onde vou guardar as minhas respostas, para este novo ciclo
            self.tempo_calculo = time.perf_counter()

            # calculo dos vários Bounding Boxes
            self.pcloud_head.robo_head() 
            for bbox in self.pcloud_head.RECEBO:

                # le os dados da BouundingBox
                # u_inicial, v_inicial, HEIGHT, WIDTH = bbox[0]
                u_inicial, v_inicial, bb_height, bb_width = bbox[0]

                # print("CENTER")
                # calcula o ponto do centro
                # this is still a change in progess. Rather than returning the x,y,z of the center point of the bounding box, 
                # we are trying to compute the mean of the points that interest us, since the center point of the bounding box may
                # be in the object/person we are tryng to get the position
                # old version - using the center of the bounding box:
                # resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + HEIGHT//2, v_inicial + WIDTH//2)
                # new version:

                resp_todos = []
                resp_todos = self.pcloud_head.converter_2D_3D(u_inicial, v_inicial, bb_height, bb_width)
                uteis = [row for row in resp_todos if (row[0]!=0 or row[1]!=0 or row[2]!=0)] # limpa os elementos [0, 0, 0]
                # print(uteis, len(uteis))
                
                if len(uteis) == 0:
                    resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
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
                        resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
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
                            resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
                        else:
                            resp_centro = centroid
                                    
                        # print("centroid:", centroid)
                        # self.get_logger().info(f"Centroid: {centroid}")

                # print("REQUESTED")
                # calcula a lista de pontos
                resp_outros = []
                for i in bbox[1]:
                    temp = self.pcloud_head.converter_2D_3D_unico(i[0], i[1])
                    resp_outros.append(temp)

                # Guarda todas as respostas na variavel ENVIO
                temp = []
                temp.append(resp_centro)
                temp.append(resp_outros)
                temp.append(uteis)
                self.pcloud_head.ENVIO.append(temp)

            # convert ENVIO into GetPointCloudBB.Response()
            ret = []
            if len(self.pcloud_head.ENVIO) > 0:
                for cc in self.pcloud_head.ENVIO:
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

            final_global_pc = PointCloudCoordinates()
            final_global_pc = response.coords[0]


            final_global_pc.bbox_point_coords = [value for index, value in enumerate(final_global_pc.bbox_point_coords) if (index+1) % 10 == 0]

            self.obstacles_head_depth_publisher.publish(final_global_pc)            

            print("n_pontos:", len(final_global_pc.bbox_point_coords))
            
            # imprime os tempos de processamento e da frame
            self.get_logger().info(f"Point Cloud Time (Head Depth): {time.perf_counter() - self.tempo_calculo}")
            # print('tempo calculo = ', time.perf_counter() - self.tempo_calculo)   # imprime o tempo de calculo em segundos
            # print('tempo frame = ', time.perf_counter() - self.tempo_frame)   # imprime o tempo de calculo em segundos
            # self.tempo_frame = time.perf_counter()
        
        else:
            # this prevents an error that sometimes on a low computational power PC that the rgb image arrives at yolo node 
            # but the depth has not yet arrived. This is a rare bug, but it crashes the yolos nodes being used.
            self.get_logger().error(f"Depth Image was not received. Please restart...")
        
        # print(response)
        # return response
        
        
    def callback_point_cloud_bb(self, request, response):

        # print(request)

        # Type of service received:
        # BoundingBoxAndPoints[] data # bounding box and specific points inside the bounding box  
        # bool retrieve_bbox # if it is intended to get the full bounding box of 3D points returned, saves data transitions  
        # string camera # which camera is being used (head or hand camera)
        # ---
        # PointCloudCoordinates[] coords # returns the selected 3D points (the bounding box center, the custom ones and the full bounding box)
        
        if request.camera == "head":

            if self.head_depth_img.height > 0 and self.head_rgb_img.height > 0: # prevents doing this code before receiving images

                # rgb_frame = self.br.imgmsg_to_cv2(self.head_rgb_img, "bgr8")
                depth_frame = self.br.imgmsg_to_cv2(self.head_depth_img, desired_encoding="passthrough")
                
                width = self.head_rgb_img.width
                height = self.head_rgb_img.height

                depth_frame_res = cv2.resize(depth_frame, (width, height), interpolation = cv2.INTER_NEAREST)

                depth_frame_res[depth_frame_res > self.pcloud_head.MAX_DIST] = 0
                depth_frame_res[depth_frame_res < self.pcloud_head.MIN_DIST] = 0

                # self.pcloud_head.rgb_img_pc = rgb_frame
                self.pcloud_head.depth_img_pc = depth_frame_res
                self.pcloud_head.RECEBO = []
                for i in range(len(request.data)):
                    aux = []
                    aux.append([request.data[i].bbox.box_top_left_y, request.data[i].bbox.box_top_left_x, request.data[i].bbox.box_height, request.data[i].bbox.box_width])

                    a = []
                    for j in range(len(request.data[i].requested_point_coords)):
                        a.append([int(request.data[i].requested_point_coords[j].y), int(request.data[i].requested_point_coords[j].x)])

                    aux.append(a)
                    self.pcloud_head.RECEBO.append(aux)

                self.pcloud_head.ENVIO = []      # limpo a variavel onde vou guardar as minhas respostas, para este novo ciclo
                self.tempo_calculo = time.perf_counter()

                # calculo dos vários Bounding Boxes
                self.pcloud_head.robo_head() 
                for bbox in self.pcloud_head.RECEBO:

                    # le os dados da BouundingBox
                    # u_inicial, v_inicial, HEIGHT, WIDTH = bbox[0]
                    u_inicial, v_inicial, bb_height, bb_width = bbox[0]

                    # print("CENTER")
                    # calcula o ponto do centro
                    # this is still a change in progess. Rather than returning the x,y,z of the center point of the bounding box, 
                    # we are trying to compute the mean of the points that interest us, since the center point of the bounding box may
                    # be in the object/person we are tryng to get the position
                    # old version - using the center of the bounding box:
                    # resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + HEIGHT//2, v_inicial + WIDTH//2)
                    # new version:

                    resp_todos = []
                    resp_todos = self.pcloud_head.converter_2D_3D(u_inicial, v_inicial, bb_height, bb_width)
                    uteis = [row for row in resp_todos if (row[0]!=0 or row[1]!=0 or row[2]!=0)] # limpa os elementos [0, 0, 0]
                    # print(uteis, len(uteis))
                    
                    if len(uteis) == 0:
                        resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
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
                            resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
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
                                resp_centro = self.pcloud_head.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
                            else:
                                resp_centro = centroid
                                        
                            # print("centroid:", centroid)
                            # self.get_logger().info(f"Centroid: {centroid}")

                    # print("REQUESTED")
                    # calcula a lista de pontos
                    resp_outros = []
                    for i in bbox[1]:
                        temp = self.pcloud_head.converter_2D_3D_unico(i[0], i[1])
                        resp_outros.append(temp)

                    # Guarda todas as respostas na variavel ENVIO
                    temp = []
                    temp.append(resp_centro)
                    temp.append(resp_outros)
                    temp.append(uteis)
                    self.pcloud_head.ENVIO.append(temp)
                    
                # convert ENVIO into GetPointCloudBB.Response()
                ret = []
                if len(self.pcloud_head.ENVIO) > 0:
                    for cc in self.pcloud_head.ENVIO:
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
                self.get_logger().info(f"Point Cloud Time (Head BB): {time.perf_counter() - self.tempo_calculo}")
                # print('tempo calculo = ', time.perf_counter() - self.tempo_calculo)   # imprime o tempo de calculo em segundos
                # print('tempo frame = ', time.perf_counter() - self.tempo_frame)   # imprime o tempo de calculo em segundos
                # self.tempo_frame = time.perf_counter()
            
            else:
                # this prevents an error that sometimes on a low computational power PC that the rgb image arrives at yolo node 
                # but the depth has not yet arrived. This is a rare bug, but it crashes the yolos nodes being used.
                self.get_logger().error(f"Depth Image was not received. Please restart...")
            
            # print(response)
            return response
        
        else:

            if self.hand_depth_img.height > 0 and self.hand_rgb_img.height > 0: # prevents doing this code before receiving images

                depth_frame = self.br.imgmsg_to_cv2(self.hand_depth_img, desired_encoding="passthrough")
                
                width = self.hand_rgb_img.width
                height = self.hand_rgb_img.height

                # depth_frame_res = cv2.resize(depth_frame, (1280, 720), interpolation = cv2.INTER_NEAREST)
                depth_frame_res = cv2.resize(depth_frame, (width, height), interpolation = cv2.INTER_NEAREST)

                depth_frame_res[depth_frame_res > self.pcloud_hand.MAX_DIST] = 0
                depth_frame_res[depth_frame_res < self.pcloud_hand.MIN_DIST] = 0

                self.pcloud_hand.depth_img_pc = depth_frame_res
            
                self.pcloud_hand.RECEBO = []
                for i in range(len(request.data)):
                    aux = []
                    aux.append([request.data[i].bbox.box_top_left_y, request.data[i].bbox.box_top_left_x, request.data[i].bbox.box_height, request.data[i].bbox.box_width])

                    a = []
                    for j in range(len(request.data[i].requested_point_coords)):
                        a.append([int(request.data[i].requested_point_coords[j].y), int(request.data[i].requested_point_coords[j].x)])

                    aux.append(a)
                    self.pcloud_hand.RECEBO.append(aux)

                self.pcloud_hand.ENVIO = []      # limpo a variavel onde vou guardar as minhas respostas, para este novo ciclo
                self.tempo_calculo = time.perf_counter()

                # calculo dos vários Bounding Boxes
                self.pcloud_hand.robo_hand() 
                for bbox in self.pcloud_hand.RECEBO:

                    # le os dados da BouundingBox
                    # u_inicial, v_inicial, HEIGHT, WIDTH = bbox[0]
                    u_inicial, v_inicial, bb_height, bb_width = bbox[0]

                    # print("CENTER")
                    # calcula o ponto do centro
                    # this is still a change in progess. Rather than returning the x,y,z of the center point of the bounding box, 
                    # we are trying to compute the mean of the points that interest us, since the center point of the bounding box may
                    # be in the object/person we are tryng to get the position
                    # old version - using the center of the bounding box:
                    # resp_centro = self.pcloud_hand.converter_2D_3D_unico(u_inicial + HEIGHT//2, v_inicial + WIDTH//2)
                    # new version:

                    resp_todos = []
                    resp_todos = self.pcloud_hand.converter_2D_3D(u_inicial, v_inicial, bb_height, bb_width)
                    uteis = [row for row in resp_todos if (row[0]!=0 or row[1]!=0 or row[2]!=0)] # limpa os elementos [0, 0, 0]
                    # print(uteis, len(uteis))
                    
                    if len(uteis) == 0:
                        resp_centro = self.pcloud_hand.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
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
                            resp_centro = self.pcloud_hand.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
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
                                resp_centro = self.pcloud_hand.converter_2D_3D_unico(u_inicial + bb_height//2, v_inicial + bb_width//2) 
                            else:
                                resp_centro = centroid
                                        
                            # print("centroid:", centroid)
                            # self.get_logger().info(f"Centroid: {centroid}")

                    # print("REQUESTED")
                    # calcula a lista de pontos
                    resp_outros = []
                    for i in bbox[1]:
                        temp = self.pcloud_hand.converter_2D_3D_unico(i[0], i[1])
                        resp_outros.append(temp)

                    # Guarda todas as respostas na variavel ENVIO
                    temp = []
                    temp.append(resp_centro)
                    temp.append(resp_outros)
                    temp.append(uteis)
                    self.pcloud_hand.ENVIO.append(temp)

                # convert ENVIO into GetPointCloudBB.Response()
                ret = []
                if len(self.pcloud_hand.ENVIO) > 0:
                    for cc in self.pcloud_hand.ENVIO:
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
                self.get_logger().info(f"Point Cloud Time (Hand BB): {time.perf_counter() - self.tempo_calculo}")
                # print('tempo calculo = ', time.perf_counter() - self.tempo_calculo)   # imprime o tempo de calculo em segundos
                # print('tempo frame = ', time.perf_counter() - self.tempo_frame)   # imprime o tempo de calculo em segundos
                # self.tempo_frame = time.perf_counter()
            
            else:
                # this prevents an error that sometimes on a low computational power PC that the rgb image arrives at yolo node 
                # but the depth has not yet arrived. This is a rare bug, but it crashes the yolos nodes being used.
                self.get_logger().error(f"Depth Image was not received. Please restart...")
            
            # print(response)
            return response


    def callback_point_cloud_mask(self, request, response):

        # print(request)

        # Type of service received:
        # MaskDetection[] data # detection segmentation mask 
        # string camera # which camera is being used (head or hand camera)
        # ---
        # PointCloudCoordinates[] coords # returns the selected 3D points
        
        if request.camera == "head":

            if self.head_depth_img.height > 0 and self.head_rgb_img.height > 0: # prevents doing this code before receiving images

                # rgb_frame = self.br.imgmsg_to_cv2(self.head_rgb_img, "bgr8")
                depth_frame = self.br.imgmsg_to_cv2(self.head_depth_img, desired_encoding="passthrough")
                
                width = self.head_rgb_img.width
                height = self.head_rgb_img.height

                depth_frame_res = cv2.resize(depth_frame, (width, height), interpolation = cv2.INTER_NEAREST)

                depth_frame_res[depth_frame_res > self.pcloud_head.MAX_DIST] = 0
                depth_frame_res[depth_frame_res < self.pcloud_head.MIN_DIST] = 0

                # self.pcloud_head.rgb_img_pc = rgb_frame
                self.pcloud_head.depth_img_pc = depth_frame_res
                self.pcloud_head.RECEBO = []

                # print(len(request.data))
                for r in request.data:
                    temp_mask = []
                    for p in r.point: # converts received mask into local coordinates and numpy array
                        p_list = []
                        p_list.append(int(p.x))
                        p_list.append(int(p.y))
                        
                        temp_mask.append(p_list)

                    self.pcloud_head.RECEBO.append(np.array(temp_mask))
                # print(self.pcloud_head.RECEBO)

                self.pcloud_head.ENVIO = []
                self.tempo_calculo = time.perf_counter()

                self.pcloud_head.robo_head()
                for mask in self.pcloud_head.RECEBO:
                    b_mask = np.zeros(depth_frame_res.shape[:2], np.uint8) # creates new empty window
                    contour = mask
                    contour = contour.astype(np.int32)
                    contour = contour.reshape(-1, 1, 2)
                    cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED) # creates mask window with just the inside pixesl of the detected objects
                    depth_frame_res_mask = depth_frame_res.copy()
                    depth_frame_res_mask[b_mask == 0] = [0]
                    self.pcloud_head.ENVIO.append(self.pcloud_head.converter_2D_3D_mask(depth_frame_res_mask))

                print(self.pcloud_head.ENVIO)

                # cv2.imshow("mask", b_mask)
                # cv2.imshow("test depth", depth_frame_res_mask)
                # cv2.waitKey(10)
    
                # convert ENVIO into GetPointCloudMask.Response()
                ret = []
                if len(self.pcloud_head.ENVIO) > 0:
                    for cc in self.pcloud_head.ENVIO:
                        
                        # print(cc)
                        pcc = PointCloudCoordinates()

                        point_c = Point()
                        point_c.x = float(cc[0])
                        point_c.y = float(cc[1])
                        point_c.z = float(cc[2])

                        pcc.center_coords = point_c

                        ret.append(pcc)

                # print(ret)
                response.coords = ret
                # imprime os tempos de processamento e da frame
                self.get_logger().info(f"Point Cloud Time (Head Mask): {time.perf_counter() - self.tempo_calculo}")

            else:
                # this prevents an error that sometimes on a low computational power PC that the rgb image arrives at yolo node 
                # but the depth has not yet arrived. This is a rare bug, but it crashes the yolos nodes being used.
                self.get_logger().error(f"Depth Image was not received. Please restart...")
            
            # print(response)
            return response
        
        else:
            
            if self.hand_depth_img.height > 0 and self.hand_rgb_img.height > 0: # prevents doing this code before receiving images

                depth_frame = self.br.imgmsg_to_cv2(self.hand_depth_img, desired_encoding="passthrough")
                
                width = self.hand_rgb_img.width
                height = self.hand_rgb_img.height

                # depth_frame_res = cv2.resize(depth_frame, (1280, 720), interpolation = cv2.INTER_NEAREST)
                depth_frame_res = cv2.resize(depth_frame, (width, height), interpolation = cv2.INTER_NEAREST)

                depth_frame_res[depth_frame_res > self.pcloud_hand.MAX_DIST] = 0
                depth_frame_res[depth_frame_res < self.pcloud_hand.MIN_DIST] = 0

                self.pcloud_hand.depth_img_pc = depth_frame_res
                self.pcloud_hand.RECEBO = []

                # print(len(request.data))
                for r in request.data:
                    temp_mask = []
                    for p in r.point: # converts received mask into local coordinates and numpy array
                        p_list = []
                        p_list.append(int(p.x))
                        p_list.append(int(p.y))
                        
                        temp_mask.append(p_list)

                    self.pcloud_hand.RECEBO.append(np.array(temp_mask))
                # print(self.pcloud_hand.RECEBO)

                self.pcloud_hand.ENVIO = []
                self.tempo_calculo = time.perf_counter()

                self.pcloud_hand.robo_hand()
                for mask in self.pcloud_hand.RECEBO:
                    b_mask = np.zeros(depth_frame_res.shape[:2], np.uint8) # creates new empty window
                    contour = mask
                    contour = contour.astype(np.int32)
                    contour = contour.reshape(-1, 1, 2)
                    cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED) # creates mask window with just the inside pixesl of the detected objects
                    depth_frame_res_mask = depth_frame_res.copy()
                    depth_frame_res_mask[b_mask == 0] = [0]
                    self.pcloud_hand.ENVIO.append(self.pcloud_hand.converter_2D_3D_mask(depth_frame_res_mask))

                print(self.pcloud_hand.ENVIO)

                # cv2.imshow("mask", b_mask)
                # cv2.imshow("test depth", depth_frame_res_mask)
                # cv2.waitKey(10)
    
                # convert ENVIO into GetPointCloudMask.Response()
                ret = []
                if len(self.pcloud_hand.ENVIO) > 0:
                    for cc in self.pcloud_hand.ENVIO:
                        
                        # print(cc)
                        pcc = PointCloudCoordinates()

                        point_c = Point()
                        point_c.x = float(cc[0])
                        point_c.y = float(cc[1])
                        point_c.z = float(cc[2])

                        pcc.center_coords = point_c

                        ret.append(pcc)

                # print(ret)
                response.coords = ret
                # imprime os tempos de processamento e da frame
                self.get_logger().info(f"Point Cloud Time (Hand Mask): {time.perf_counter() - self.tempo_calculo}")

            else:
                # this prevents an error that sometimes on a low computational power PC that the rgb image arrives at yolo node 
                # but the depth has not yet arrived. This is a rare bug, but it crashes the yolos nodes being used.
                self.get_logger().error(f"Depth Image was not received. Please restart...")

            # print(response)
            return response


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    rclpy.spin(node)
    rclpy.shutdown()
    """