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
        point3d = self.convert_mask_to_3dpoint(depth_img, camera, mask)

        return point3d
    
    def convert_pose_keypoints_to_3d_point(self, depth_img, camera, keypoints, min_kp_conf_value):

        # aaa = time.time()

        match camera:
            case "head":
                camera_used = self.head_camera
            case "hand":
                camera_used = self.hand_camera
            case "base":
                camera_used = self.base_camera

        NOSE_KP = 0
        EYE_LEFT_KP = 1                        
        EYE_RIGHT_KP = 2
        EAR_LEFT_KP = 3
        EAR_RIGHT_KP = 4
        SHOULDER_LEFT_KP = 5
        SHOULDER_RIGHT_KP = 6
        ELBOW_LEFT_KP = 7
        ELBOW_RIGHT_KP = 8
        WRIST_LEFT_KP = 9
        WRIST_RIGHT_KP = 10
        HIP_LEFT_KP = 11
        HIP_RIGHT_KP = 12
        KNEE_LEFT_KP = 13
        KNEE_RIGHT_KP = 14
        ANKLE_LEFT_KP = 15
        ANKLE_RIGHT_KP = 16

        masks = []

        masks.append(self.convert_keypopints_to_mask(keypoints, [NOSE_KP, EYE_LEFT_KP, EYE_RIGHT_KP], min_kp_conf_value))
        masks.append(self.convert_keypopints_to_mask(keypoints, [NOSE_KP, EYE_LEFT_KP, EAR_LEFT_KP], min_kp_conf_value))
        masks.append(self.convert_keypopints_to_mask(keypoints, [NOSE_KP, EYE_RIGHT_KP, EAR_RIGHT_KP], min_kp_conf_value))

        masks.append(self.convert_keypopints_to_mask(keypoints, [SHOULDER_LEFT_KP, SHOULDER_RIGHT_KP, HIP_RIGHT_KP, HIP_LEFT_KP], min_kp_conf_value))

        masks.append(self.convert_keypopints_to_mask(keypoints, [SHOULDER_LEFT_KP, ELBOW_LEFT_KP], min_kp_conf_value))
        masks.append(self.convert_keypopints_to_mask(keypoints, [ELBOW_LEFT_KP, WRIST_LEFT_KP], min_kp_conf_value))
        masks.append(self.convert_keypopints_to_mask(keypoints, [SHOULDER_RIGHT_KP, ELBOW_RIGHT_KP], min_kp_conf_value))
        masks.append(self.convert_keypopints_to_mask(keypoints, [ELBOW_RIGHT_KP, WRIST_RIGHT_KP], min_kp_conf_value))

        masks.append(self.convert_keypopints_to_mask(keypoints, [HIP_LEFT_KP, KNEE_LEFT_KP], min_kp_conf_value))
        masks.append(self.convert_keypopints_to_mask(keypoints, [KNEE_LEFT_KP, ANKLE_LEFT_KP], min_kp_conf_value))
        masks.append(self.convert_keypopints_to_mask(keypoints, [HIP_RIGHT_KP, KNEE_RIGHT_KP], min_kp_conf_value))
        masks.append(self.convert_keypopints_to_mask(keypoints, [KNEE_RIGHT_KP, ANKLE_RIGHT_KP], min_kp_conf_value))
        
        b_mask = np.zeros(depth_img.shape[:2], np.uint8) # creates new empty window
        
        for m in masks:
            if m.size > 0:
                contour = m
                contour = contour.astype(np.int32)
                contour = contour.reshape(-1, 1, 2)
                cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED) # creates mask window with just the inside pixesl of the detected objects
            
        depth_frame_res_mask = depth_img.copy()
        depth_frame_res_mask[b_mask == 0] = [0]
        # v2.imwrite('output_image.png', depth_frame_res_mask)
        # cv2.imwrite('output_image1.png', b_mask)

        ### OVERALL THE SAME PROCESS IS USED AS IN MASK, HOWEVER HERE WE HAVE A SPECIAL CASE:
        ### IS NOT NOT SENT A MASK BUT A LIST OF MASKS OF ALL THE CONNECTIONS BETWEEN KP
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
    
    
    def convert_keypopints_to_mask(self, keypoints, list_of_kp_for_mask, min_kp_conf_value):
        
        filtered_list_of_kp_for_mask = []
        mask_list = []

        for kp in list_of_kp_for_mask:
            if keypoints.conf[0][kp] > min_kp_conf_value:
                filtered_list_of_kp_for_mask.append(kp)

        if len(filtered_list_of_kp_for_mask) >= 2:

            # goes through all elements
            for kp in filtered_list_of_kp_for_mask:
                x = int(keypoints.xy[0][kp][0])
                y = int(keypoints.xy[0][kp][1])
                mask_list.append([x, y])

            # adds the first element again
            x = int(keypoints.xy[0][filtered_list_of_kp_for_mask[0]][0])
            y = int(keypoints.xy[0][filtered_list_of_kp_for_mask[0]][1])
            mask_list.append([x, y])
        # otherwise returns an empty mask which is removed whn checking .size > 0

        mask = np.array(mask_list)
        return mask
