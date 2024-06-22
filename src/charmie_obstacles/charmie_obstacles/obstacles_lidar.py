#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from charmie_interfaces.msg import ObstacleInfo, Obstacles
from example_interfaces.msg import Bool

import cv2
import numpy as np
import math
import time

class ObstaclesLIDAR:

    def __init__(self):
        print("Executing Main Code")
        
        self.START_DEG = -119.885
        self.STEP_DEG = 0.35208516886930985
        self.scale = 0.12*1000
        self.NumberOfLasers = 681
        
        self.valores_dict = {}
        self.valores_id = {}

        self.obstacles_pub = Obstacles()
        # self.obstacles_pub_ant = Obstacles()

        self.xc = 400
        self.yc = 400

        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.test_image2 = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)

        self.DEBUG_DRAW_IMAGE = True
        self.DEBUG_PRINT = True
        self.is_TRFilter = True         
        self.is_dummy_points = True
        self.is_obs = True
        self.error_lidar_reading = False
        
        #dists
        self.OBS_THRESH = 1.0
        self.min_dist_error = 0.1
        self.max_dist_error = 5.0
        self.tr_aux = 0.01
        self.DIST_MIN_THRESH = 0.1
        self.lidar_to_robot_center = 0.255
        self.robot_radius = 0.560/2
        self.lidar_radius = 0.050/2


        self.stop_image_for_debug = False # variable so that we do a 1 second delay in a frame when we are debuging

    def lidar_readings_to_obstacles(self, ls: LaserScan):
        # readings = scan.ranges
        self.lidar_dicts(ls.ranges)

        if self.valores_dict is not None:
            self.error_lidar_reading = False

            if self.DEBUG_DRAW_IMAGE:
                self.test_image[:, :] = 0  # limpa a imagem
                self.test_image2[:, :] = 0  # limpa a imagem

            if self.is_TRFilter:
                self.TRFilter()

            obst_list = self.draw_points_and_obstacle_detection() 
            obs_dict_v = self.obstacle_detection(obst_list) 


            # print(self.valores_id)
            # print(self.valores_dict)
            if self.DEBUG_DRAW_IMAGE:
                cv2.circle(self.test_image, (self.xc, self.yc), (int)(self.OBS_THRESH*self.scale), (0, 255, 255), 1)
                cv2.line(self.test_image2, (60, (int)(self.yc + 100 - self.OBS_THRESH*self.scale)), (800-60+1, (int)(self.yc + 100 - self.OBS_THRESH*self.scale)), (0, 255, 255), 1)
                
                # corpo lidar
                cv2.circle(self.test_image, (self.xc, self.yc), (int)(self.lidar_radius*self.scale), (255, 255, 255), 1)
                # centro robo
                cv2.circle(self.test_image, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)), (int)(self.robot_radius*self.scale/10), (255, 255, 255), 1)
                # corpo robo
                cv2.circle(self.test_image, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)), (int)(self.robot_radius*self.scale), (255, 255, 255), 1)
                
                # cv2.imshow("LIDAR Linear", self.test_image2)
                cv2.imshow("LIDAR Circular", self.test_image)

                # if self.stop_image_for_debug:
                #     self.stop_image_for_debug = False
                #     time.sleep(5)
                
                k = cv2.waitKey(1)
                if k == ord('+'):
                    self.scale /= 0.8
                if k == ord('-'):
                    self.scale *= 0.8

            # return self.data_ready_to_publish(obs_dict_v)
            # self.obstacles_pub_ant = self.obstacles_pub
            self.obstacles_pub = self.data_ready_to_publish(obs_dict_v)
            # return self.obstacles_pub
        else:
            self.error_lidar_reading = True
            print("ERRO DO LIDAR AO ENVIAR TRAMA!!!!!!!!!! Segui com a minha vida...")
            
            # if it does not have new obstacle values to be updated than it does not update  


    def lidar_dicts(self, readings):
        # print(readings)
        # print(len(readings))
        # print("GO")
        # print(self.START_DEG)

        # do 0 and valor que esta no range -1 
        # self.valores_dict_ant = self.valores_dict
        for i in range(len(readings)):
            # print(x)
            # i = i + 1
            self.valores_id[self.START_DEG+i*self.STEP_DEG] = i
            self.valores_dict[self.START_DEG+i*self.STEP_DEG] = readings[i]

        # print(self.valores_id)
        # print(self.valores_dict)

        self.centre_data = (int)((self.test_image.shape[1] - 682) / 2)


    def TRFilter(self):


        # aux = self.OBS_THRESH + self.tr_aux 
        # for key, value in self.valores_id.items():
        #     if self.valores_dict[key] < self.min_dist_error or self.valores_dict[key] > self.max_dist_error:
        #         self.valores_dict[key] = aux
        #     else:
        #         aux = self.valores_dict[key]

        # aux = self.OBS_THRESH + self.tr_aux 
        for key, value in self.valores_id.items():
            if self.valores_dict[key] < self.min_dist_error or self.valores_dict[key] > self.max_dist_error:
                self.valores_dict[key] = 8
            # else:
            #     aux = self.valores_dict[key]

    def draw_points_and_obstacle_detection(self):  

        obst_dict = {}
        obst_list = []
        dist_ant = 0
        value_ant = 0
        key_ant = 0
        is_obstacle = False
        dist_counter = 0
        dist_min = self.OBS_THRESH
        dist_sum = 0

        # line used to know the zero value of the derivatives
        if self.DEBUG_DRAW_IMAGE:
            cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - 0, self.yc + 250), (self.test_image.shape[1] - self.centre_data - self.NumberOfLasers, int(self.yc + 250 + self.scale * 0.5 * 0)), (255, 0, 0))


        for key, value in self.valores_id.items():
            # print(value, ":", key, ":", self.valores_dict[key])

            # ### key is the reading ID
            # ### value is the reading angle
            # ### valores_dict[value] is the reading distance
    
            # draws two LIDAR graphs, the circular one and the linear one
            # from right to left
            if self.DEBUG_DRAW_IMAGE:
                if self.valores_dict[key] > self.OBS_THRESH:
                    cv2.line(self.test_image, (self.xc, self.yc), (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))),
                                                    int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))),
                             (255, 0, 0))
                    cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 100),
                             (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 100 - self.scale * self.valores_dict[key])),
                             (255, 0, 0))
                else:
                    cv2.line(self.test_image, (self.xc, self.yc), (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))),
                                                    int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))),
                             (0, 0, 255))
                    cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 100),
                             (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 100 - self.scale * self.valores_dict[key])),
                             (0, 0, 255))
        
            # #### V2.0 #### #
            if value == 0:  # first angle case, since there is no previous value
                if self.valores_dict[key] < self.OBS_THRESH:
                    # print(value, ":", key, ":", valores_dict[key])
                    if self.DEBUG_DRAW_IMAGE:
                        cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 250 + self.scale * 0.5 * 0.001 * + (500))), (255, 0, 255))
                        if self.is_dummy_points:
                            cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))), int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))), (int)(2), (0, 100, 255), 1)
                            cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key], int(self.yc + 100 - self.scale * self.valores_dict[key])), (int)(2), (0, 100, 255), 1)
                        # print("Start Obstacle: (", key, "-", valores_dict[key], ")")
                    obst_dict['alfa_i'] = key
                    obst_dict['d_i'] = self.valores_dict[key]
                    dist_min = self.valores_dict[key]
                    # key
                    is_obstacle = True
    
            if value > 0:
                if self.valores_dict[key] < self.OBS_THRESH and dist_ant < self.OBS_THRESH:
                    # print(value, ":", key, ":", valores_dict[key])
    
                    if self.valores_dict[key] - dist_ant > self.DIST_MIN_THRESH:  # the new obstacle is further
                        if self.DEBUG_DRAW_IMAGE:
                            cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 250 + self.scale * 0.5 * 0.001 * -(700))), (0, 255, 255))
                            if self.is_dummy_points:
                                cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))), int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))), (int)(2), (255, 255, 0), 1)
                                cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key], int(self.yc + 100 - self.scale * self.valores_dict[key])), (int)(2), (255, 255, 0), 1)
                            # print("Middle Obstacle: (", key, "-", valores_dict[key], ")")
                        obst_dict['alfa_f'] = key_ant
                        obst_dict['d_f'] = self.valores_dict[key_ant] 
                        if dist_counter == 0:
                            obst_dict['d_avg'] = dist_min # BUG on 02/07/2023, must solve later
                        else:
                            obst_dict['d_avg'] = dist_sum/dist_counter
                        obst_dict['d_min'] = dist_min
                        obst_list.append(obst_dict.copy())  # for some reason it needs the copy to work
                        obst_dict.clear()
                        dist_sum = 0
                        dist_counter = 0
                        dist_min = self.OBS_THRESH
                        obst_dict['alfa_i'] = key
                        obst_dict['d_i'] = self.valores_dict[key]
    
                    if self.valores_dict[key] - dist_ant < -self.DIST_MIN_THRESH:  # the new obstacle is closer
                        if self.DEBUG_DRAW_IMAGE:
                            cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 250 + self.scale * 0.5 * 0.001 * +(300))), (0, 255, 255))
                            if self.is_dummy_points:
                                cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))), int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))), (int)(2), (255, 255, 0), 1)
                                cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key], int(self.yc + 100 - self.scale * self.valores_dict[key])), (int)(2), (255, 255, 0), 1)
                            # print("Middle Obstacle: (", key, "-", valores_dict[key], ")")
                        obst_dict['alfa_f'] = key_ant
                        obst_dict['d_f'] = self.valores_dict[key_ant]
                        if dist_counter == 0:
                            obst_dict['d_avg'] = dist_min # BUG on 02/07/2023, must solve later
                        else:
                            obst_dict['d_avg'] = dist_sum/dist_counter
                        obst_dict['d_min'] = dist_min
                        obst_list.append(obst_dict.copy())  # for some reason it needs the copy to work
                        obst_dict.clear()
                        dist_sum = 0
                        dist_counter = 0
                        dist_min = self.OBS_THRESH
                        obst_dict['alfa_i'] = key
                        obst_dict['d_i'] = self.valores_dict[key]

                if self.valores_dict[key] > self.OBS_THRESH >= dist_ant and is_obstacle:  # from right to left, end of obstacle
                    # print(value, ":", key, ":", valores_dict[key])
                    if self.DEBUG_DRAW_IMAGE:
                        cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value_ant, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value_ant, int(self.yc + 250 + self.scale * 0.5 * 0.001 * -(500))), (255, 255, 255))
                        if self.is_dummy_points:
                            cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key_ant] * math.cos(math.radians(-key_ant + 90))), int(self.yc - self.scale * self.valores_dict[key_ant] * math.sin(math.radians(-key_ant + 90)))), (int)(3), (255, 180, 180), 1)
                            cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key_ant], int(self.yc + 100 - self.scale * self.valores_dict[key_ant])), (int)(3), (255, 180, 180), 1)
                        # print("End Obstacle: (", key_ant, "-", valores_dict[key_ant], ")")
                    obst_dict['alfa_f'] = key_ant
                    obst_dict['d_f'] = self.valores_dict[key_ant]
                    if dist_counter == 0:
                        obst_dict['d_avg'] = dist_min # BUG on 02/07/2023, must solve later
                    else:
                        obst_dict['d_avg'] = dist_sum/dist_counter
                    obst_dict['d_min'] = dist_min
                    obst_list.append(obst_dict.copy())  # for some reason it needs the copy to work
                    obst_dict.clear()
                    is_obstacle = False
    
                if self.valores_dict[key] <= self.OBS_THRESH < dist_ant:  # from right to left, start of obstacle
                    # print(value, ":", key, ":", valores_dict[key])
                    if self.DEBUG_DRAW_IMAGE:
                        cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 250 + self.scale * 0.5 * 0.001 * +(500))), (255, 0, 255))
                        if self.is_dummy_points:
                            cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))), int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))), (int)(2), (0, 100, 255), 1)
                            cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key], int(self.yc + 100 - self.scale * self.valores_dict[key])), (int)(2), (0, 100, 255), 1)
                        # print("Start Obstacle: (", key, "-", valores_dict[key], ")")
                    obst_dict['alfa_i'] = key
                    obst_dict['d_i'] = self.valores_dict[key]
                    dist_min = self.valores_dict[key]
                    is_obstacle = True
                    # key
    
            if value == self.NumberOfLasers:  # last angle case, since there is no next value
                if self.valores_dict[key] < self.OBS_THRESH:
                    # print(value, ":", key, ":", valores_dict[key])
                    if self.DEBUG_DRAW_IMAGE:
                        cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - value, self.yc + 250), (self.test_image.shape[1] - self.centre_data - value, int(self.yc + 250 + self.scale * 0.5 * 0.001 * -(500))), (255, 255, 255))
                        if self.is_dummy_points:
                            cv2.circle(self.test_image, (int(self.xc - self.scale * self.valores_dict[key] * math.cos(math.radians(-key + 90))), int(self.yc - self.scale * self.valores_dict[key] * math.sin(math.radians(-key + 90)))), (int)(3), (255, 180, 180), 1)
                            cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[key], int(self.yc + 100 - self.scale * self.valores_dict[key])), (int)(3), (255, 180, 180), 1)
                        # print("End Obstacle: (", key, "-", valores_dict[key], ")")
                    obst_dict['alfa_f'] = key
                    obst_dict['d_f'] = self.valores_dict[key]
                    dist_sum += self.valores_dict[key]
                    dist_counter += 1
                    obst_dict['d_avg'] = dist_sum / dist_counter
                    obst_dict['d_min'] = dist_min
                    obst_list.append(obst_dict.copy())  # for some reason it needs the copy to work
                    obst_dict.clear()
                    is_obstacle = False
                    # key_ant

            # defines when a new obstacle starts and helps computing the distance average and minimum value
            if is_obstacle:
                dist_sum += self.valores_dict[key]
                dist_counter += 1
                if self.valores_dict[key] < dist_min:
                    dist_min = self.valores_dict[key]
            else:
                dist_sum = 0
                dist_counter = 0
                dist_min = self.OBS_THRESH
    
            value_ant = value
            dist_ant = self.valores_dict[key]
            key_ant = key
    
        # for obs in obst_list:
        #     print(obs)
    
        # print(obst_list)
        return obst_list
        
    def obstacle_detection(self, obst_list):

        # # # obst_dict is a list of dicts, each dict is an obstacle with the following parameters:
        # alfa_i: starting angle of the obstacle
        # d_i:    distance of the starting angle of the obstacle
        # alfa_i: final angle of the obstacle
        # d_i:    distance of the final angle of the obstacle
        # d_avg:  average of the obstacle distances
        # d_min:  minimum of the obstacle distances

        obs_list_v = []
        obstacle = {}
        obstacle_ = {}
        # obstacle2 = {}

        print("START OF CODE BEING TESTED:")
        print(obst_list)

        ########## ORA BEM, ESTÁ NA HORA DE FAZER MILAGRES ANTES DO ROBOCUP 23 ;) DG STYLE
        # função para dividir obstaculos grande em obstaculos pequenos. A maneira como o SDNL precisa dos dados para reconhecer obstaculos,
        # faz com que paredes laterais ao robô sejam vistas como 45º para a frente do robô, o que estraga tudo...
        # ou seja, está na hora de numa direta resolver isto, convém depois do RoboCup ver se realmente isto ficou mesmo direitinho...
         
        cut_obst_list = []

        obst_list_aux = obst_list

        cutting_obs_dist_cm = 0.1
        cutting_obs_dist_deg = 8 # was 5
        # idealmente isto teria que ser feito com o len_cm mas é muito mais rapido de implementar com diferencças de graus, para já vou tentar assim...

        for obs in obst_list_aux:
            x1 = obs['d_i'] * math.cos(math.radians(-obs['alfa_i'] + 90 - self.STEP_DEG))
            y1 = obs['d_i'] * math.sin(math.radians(-obs['alfa_i'] + 90 - self.STEP_DEG))
            x2 = obs['d_f'] * math.cos(math.radians(-obs['alfa_f'] + 90 + self.STEP_DEG))
            y2 = obs['d_f'] * math.sin(math.radians(-obs['alfa_f'] + 90 + self.STEP_DEG))
            l = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            aux_length_deg = - (obs['alfa_i'] - obs['alfa_f'])
            print("len_deg:", aux_length_deg)
            

            a_i = obs['alfa_i']
            d_i = obs['d_i']
            a_f = obs['alfa_f']
            d_f = obs['d_f']

            closer_obs = {}

            if aux_length_deg > cutting_obs_dist_deg:
                
                while a_i !=  obs['alfa_f']:
                    
                    temp_cut_obs_list = []
                    print("INSIDE")

                    # teta_entre_extremos1 = math.atan2(y2-y1, x2-x1)
                    # teta_entre_extremos2 = math.atan2(y1-y2, x1-x2)

                    teta_novo = a_i + cutting_obs_dist_deg

                    print(teta_novo)

                    # procura o valor no dicionario mais perto e devolve esse valor, esse valor é o novo a_f desta iteração e o a_i da proxima


                    aux_erro = 1000
                    next_key = 0.0
                    next_val = 0.0
                    for key, value in self.valores_dict.items():
                        err = abs(teta_novo - key)
                        if err < aux_erro:
                            aux_erro = err
                            next_key = key
                            next_val = value

                    if next_key > obs['alfa_f']:
                        a_f = obs['alfa_f']
                        d_f = obs['d_f']
                    else:
                        a_f = next_key
                        d_f = next_val

                    # print(next_key, next_val)
                    # print("OUT")

                    new_obs = {}
                    new_obs['alfa_i'] = a_i
                    new_obs['d_i'] = d_i
                    new_obs['alfa_f'] = a_f
                    new_obs['d_f'] = d_f
                    # fiz isto para ser rapido, é possivel que seja fazer melhor!!!
                    new_obs['d_avg'] = (d_i+d_f)/2
                    new_obs['d_min'] = d_i

                    
                    # cut_obst_list.append(new_obs)
                    if not closer_obs:
                        closer_obs = new_obs
                    else:
                        if new_obs['d_avg'] < closer_obs['d_avg']:
                            closer_obs = new_obs
                    

                    a_i = a_f
                    d_i = d_f
                    # a_f = obs['alfa_f']
                    # d_f = obs['d_f']
                    aux_length_deg = - (a_i - a_f)

                

                cut_obst_list.append(closer_obs)

            else:
                cut_obst_list.append(obs)
                

        print("CUT_VERSION:", cut_obst_list)


        for obs in cut_obst_list:
        # for obs in obst_list:
            # obstacle2['alfa'] = obs['alfa_i'] + (obs['alfa_f'] - obs['alfa_i']) / 2

            # 2*STEP_DEG to guarantee that the blind stop between rays is included in the object width
            # this method uses the distance values of the first and last angle to calculate the length of the obstacle
            # however some materials (i.e: metal) when detected have high variance on the first and last distances
            # thus this method has high variance and intrinsic error
            # _x1 = valores_dict[obs['alfa_i']] * math.cos(math.radians(-obs['alfa_i'] + 90 - STEP_DEG))
            # _y1 = valores_dict[obs['alfa_i']] * math.sin(math.radians(-obs['alfa_i'] + 90 - STEP_DEG))
            # _x2 = valores_dict[obs['alfa_f']] * math.cos(math.radians(-obs['alfa_f'] + 90 + STEP_DEG))
            # _y2 = valores_dict[obs['alfa_f']] * math.sin(math.radians(-obs['alfa_f'] + 90 + STEP_DEG))
            # _l = math.sqrt((_x2 - _x1) ** 2 + (_y2 - _y1) ** 2)

            # 2*STEP_DEG to guarantee that the blind stop between rays is included in the object width
            # this method uses the average distance of the obstacle to calculate its length,
            # thus it has less variance in error
            # however the length value always tends to be a bit shy of the true value
            x1 = obs['d_avg'] * math.cos(math.radians(-obs['alfa_i'] + 90 - self.STEP_DEG))
            y1 = obs['d_avg'] * math.sin(math.radians(-obs['alfa_i'] + 90 - self.STEP_DEG))
            x2 = obs['d_avg'] * math.cos(math.radians(-obs['alfa_f'] + 90 + self.STEP_DEG))
            y2 = obs['d_avg'] * math.sin(math.radians(-obs['alfa_f'] + 90 + self.STEP_DEG))
            l = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            obstacle['alfa'] = (obs['alfa_i'] + obs['alfa_f']) / 2  # center angle, just the average of the obstacle start and end angle
            obstacle['dist'] = obs['d_avg']  # it can also be used the minimum distance
            obstacle['length_cm'] = l
            obstacle['length_deg'] = - (obs['alfa_i'] - obs['alfa_f'])
            
            
            
            
            
            # print(obs, obstacle)

            #                  .(OBS)
            #                //
            #              /A/ 
            #          b /  /
            #          /   /
            #  (LID)./    /  
            #       |C   / c
            #       |   /
            #     a |  /
            #       |B/
            #       |/
            #  (ROB)'
            # 
            # Use the Law of Cosines to find one of the remaining angles. 
            # The Law of Cosines states that for any triangle with sides a, b, and c, and the opposite angle C, 
            # the following equation holds:
            # c^2 = a^2 + b^2 - 2ab * cos(C)          


            #                  .(OBS)
            #                //
            #              /A/ 
            #       dl_o /  /
            #          /   /
            #  (LID)./    /  
            #       |a   / dr_o
            #       |   /
            #  dl_r |  /
            #       |b/
            #       |/
            #  (ROB)'
            # 
            # c = sqrt(a^2 + b^2 - 2abcosC)   
            # dr_o = sqrt(dl_r^2 + dl_o^2 - 2*dl_r*dl_o*cos(alfa))  
            # 
            # C = cos-1((a^2+b^2-c^2)/2ab)   
            # beta = acos((dl_r^2+dr_o^2-dl_o^2)/2*dl_r*dr_o)   


            # obstacle['dist'] = 0.884190471399398
            # obstacle['alfa'] = 1.26280683878509

            obstacle_['dist'] = math.sqrt(self.lidar_to_robot_center*self.lidar_to_robot_center + obstacle['dist']*obstacle['dist'] -
                                          2*self.lidar_to_robot_center*obstacle['dist']*math.cos(math.pi-math.radians(obstacle['alfa'])))
            
            # prevents crashes that happened previously due to roundings fo 1.0 to 1.0000000000000004 (pasted from terminal) which are
            # outside the scope of acos which is between -1 and 1, inclusive
            aux = (self.lidar_to_robot_center*self.lidar_to_robot_center + obstacle_['dist']*obstacle_['dist'] -
                                                obstacle['dist']*obstacle['dist'])/(2*self.lidar_to_robot_center*obstacle_['dist'])
            
            
            # print("old =", obstacle['alfa'])
            # with this function I force the value to 1.0 if it rounds up to bigger than 1.0
            # it is not necessary to do this for -1, since i cannot detect obstacles in the back of the robot
            # The 1.0 input to the arcos is for angle 0 (front of the robot), the -1 is for angle 180 (deg) so the back of the robot
            if aux > 1.0:
                if self.DEBUG_PRINT:
                    print("Input of acos > 1.0 prevented!")
                aux = 1.0
                # self.stop_image_for_debug = True

            # however there is still a visual bug, due to roundings, check rosbag. (/charmie_ws/rosbags/erro_acos)
            # what can be done is just check the distance of both points in the circle and just draw in the image the one that is nearer.
            # OU DESENHAR DE MANEIRA DIFERENTE porque se o angulo e a distancia estão certos porque é que preciso de calcular os pontos, 
            # nao posso so desenhar a partir do centro e "apagar o resto" devo estar a fazer algo que não é a maneira mais eficiente

            if obstacle['alfa'] > 0:
                obstacle_['alfa'] = +math.acos(aux)
            else:
                obstacle_['alfa'] = -math.acos(aux)
            

            obstacle_['length_cm'] = obstacle['length_cm']
            obstacle_['length_deg'] = obstacle['length_deg']
            

            obstacle_['alfa'] = math.degrees(obstacle_['alfa'])
            # print("new =", obstacle_['alfa'])
            # print("dist =", obstacle_['dist'])
            

            if self.DEBUG_DRAW_IMAGE:
                if self.is_obs:
                    
                    # OLD CALCULATION GREEN LINES
                    # cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90))),
                    #                         int(self.yc - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)))),
                    #            (int)(2), (0, 255, 0), 1)
                    # cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90)) + self.scale * (obstacle['length_cm']/2) * math.cos(math.radians(-(90 - obstacle['alfa']) + 90))),
                    #                         int(self.yc - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)) - self.scale * (obstacle['length_cm']/2) * math.sin(math.radians(-(90 - obstacle['alfa']) + 90)))),
                    #            (int)(2), (0, 255, 0), 1)
                    # cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90)) - self.scale * (obstacle['length_cm']/2) * math.cos(math.radians(-(90 - obstacle['alfa']) + 90))),
                    #                         int(self.yc - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)) + self.scale * (obstacle['length_cm']/2) * math.sin(math.radians(-(90 - obstacle['alfa']) + 90)))),
                    #           (int)(2), (0, 255, 0), 1)
                    # line lidar center to obstacle center
                    # cv2.line(self.test_image, (self.xc, self.yc), (int(self.xc - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90))),
                    #                                                int(self.yc - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)))),
                    #           (0, 255, 0))
                    
                    
                    cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90))),
                                                 int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90)))),
                                                (int)(2), (255, 255, 255), 1)
                    cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90)) + self.scale * (obstacle_['length_cm']/2) * math.cos(math.radians(-(90 - obstacle_['alfa']) + 90))),
                                                 int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90)) - self.scale * (obstacle_['length_cm']/2) * math.sin(math.radians(-(90 - obstacle_['alfa']) + 90)))),
                                                (int)(2), (255, 255, 255), 1)
                    cv2.circle(self.test_image, (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90)) - self.scale * (obstacle_['length_cm']/2) * math.cos(math.radians(-(90 - obstacle_['alfa']) + 90))),
                                                 int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90)) + self.scale * (obstacle_['length_cm']/2) * math.sin(math.radians(-(90 - obstacle_['alfa']) + 90)))),
                                                (int)(2), (255, 255, 255), 1)
                    cv2.line(self.test_image, (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90)) + self.scale * (obstacle_['length_cm']/2) * math.cos(math.radians(-(90 - obstacle_['alfa']) + 90))),
                                               int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90)) - self.scale * (obstacle_['length_cm']/2) * math.sin(math.radians(-(90 - obstacle_['alfa']) + 90)))),
                                              (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90)) - self.scale * (obstacle_['length_cm']/2) * math.cos(math.radians(-(90 - obstacle_['alfa']) + 90))),
                                               int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90)) + self.scale * (obstacle_['length_cm']/2) * math.sin(math.radians(-(90 - obstacle_['alfa']) + 90)))),
                                              (255, 255, 255))
                    
                                    
                    id_alfa = int((self.valores_id[obs['alfa_i']] + self.valores_id[obs['alfa_f']]) / 2)

                    cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - id_alfa, int(self.yc + 100 - self.scale * obstacle['dist'])), (int)(2), (0, 255, 0), 1)
                    cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[obs['alfa_i']] + 1, int(self.yc + 100 - self.scale * obstacle['dist'])), (int)(2), (0, 255, 0), 1)
                    cv2.circle(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[obs['alfa_f']] - 1, int(self.yc + 100 - self.scale * obstacle['dist'])), (int)(2), (0, 255, 0), 1)
                    cv2.line(self.test_image2, (self.test_image.shape[1] - self.centre_data - self.valores_id[obs['alfa_i']] + 1, int(self.yc + 100 - self.scale * obstacle['dist'])),
                                          (self.test_image.shape[1] - self.centre_data - self.valores_id[obs['alfa_f']] - 1, int(self.yc + 100 - self.scale * obstacle['dist'])),
                             (0, 255, 0))
                    

                    # y = mx + b
                    (x1, y1) = (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale))
                    (x2, y2) = (int(self.xc - self.scale * obstacle_['dist'] * math.cos(math.radians(-obstacle_['alfa'] + 90))),
                                int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * obstacle_['dist'] * math.sin(math.radians(-obstacle_['alfa'] + 90))))

                    # to prevent divisions by zero
                    
                    if x1 == x2:
                        x2+=1
                        if self.DEBUG_PRINT:
                            print("Division by 0 prevented!")
                    
                    m = (y2-y1)/(x2-x1)
                    b = y1 - (m*x1)

                    # (x - h)^2 + (y - k)^2 = r^2
                    (h, k) = (x1, y1)
                    r = (int)(self.robot_radius*self.scale)

                    # intersection
                    # (x - h)^2 + (mx + c - k)^2 = r^2
                    # simplified
                    # (1 + m^2)x^2 + (2mc - 2hk)x + (h^2 + c^2 - 2mkx + 2kcx + k^2 - r^2) = 0

                    # (1 + m^2)x^2 + (2bm - 2h)x + (h^2 + b^2 - 2bk + k^2 - r^2) = 0
                    A = 1 + m*m
                    B = -2*h + 2*m*b - 2*m*k
                    # B = -2*h + 2*m*b - 2*m*k
                    C = h*h + b*b - 2*k*b + k*k - r*r


                    x3 = (-B + math.sqrt((B ** 2) - (4 * A * C))) / (2 * A)
                    x4 = (-B - math.sqrt((B ** 2) - (4 * A * C))) / (2 * A)

                    y3 = m*x3 + b
                    y4 = m*x4 + b

                    # print(x3, y3)
                    # print(x4, y4)

                    if obstacle_['alfa'] < 0:
                        pi_x = x3
                        pi_y = y3
                    else:
                        pi_x = x4
                        pi_y = y4
                
                    # line lidar center to obstacle center
                    # cv2.line(self.test_image, (self.xc, self.yc), (int(self.xc - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90))),
                    #                                                int(self.yc - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)))),
                    #           (0, 255, 0))
                    
                    #line robot center to obstacle center
                    # cv2.line(self.test_image, (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale)),
                    #           (int(self.xc - self.scale * (obstacle_['dist']+self.robot_radius) * math.cos(math.radians(-obstacle_['alfa'] + 90))),
                    #            int(self.yc + self.lidar_to_robot_center*self.scale - self.scale * (obstacle_['dist']+self.robot_radius) * math.sin(math.radians(-obstacle_['alfa'] + 90)))),
                    #           (255, 255, 255))
                    
                    
                    # Visual Bug: Does not influence the working of the robot, because of angle 0 acos > 1.0
                    # explained when calculating acos

                    
                    # ponto de interseção da linha distancia do centro do robo ao obstaculo com o raio do robo
                    cv2.circle(self.test_image, (int(pi_x), int(pi_y)), (int)(4), (255, 0, 255), -1)
 
                    #line robot platform to obstacle center
                    cv2.line(self.test_image, (int(pi_x), int(pi_y)),
                              (int(pi_x - self.scale * (obstacle_['dist']-self.robot_radius) * math.cos(math.radians(-obstacle_['alfa'] + 90))),
                               int(pi_y - self.scale * (obstacle_['dist']-self.robot_radius) * math.sin(math.radians(-obstacle_['alfa'] + 90)))),
                              (255, 255, 255))
                    



                    # erro antigo em que:
                    # 1) valores estao centradas no lidar e não no robo
                    # 2) é assumido que os valores estão todos a sair da margem do robo e nao do centro 

                    # y = mx + b
                    (x1, y1) = (self.xc, int(self.yc+self.lidar_to_robot_center*self.scale))
                    (x2, y2) = (int(self.xc                                       - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90))),
                                int(self.yc+self.lidar_to_robot_center*self.scale - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90))))
                    
                    if x1 == x2:
                        x2+=1
                        if self.DEBUG_PRINT:
                            print("Division by 0 prevented!")
                    
                    m = (y2-y1)/(x2-x1)
                    b = y1 - (m*x1)

                    # (x - h)^2 + (y - k)^2 = r^2
                    (h, k) = (x1, y1)
                    r = (int)(self.robot_radius*self.scale)

                    # intersection
                    # (x - h)^2 + (mx + c - k)^2 = r^2
                    # simplified
                    # (1 + m^2)x^2 + (2mc - 2hk)x + (h^2 + c^2 - 2mkx + 2kcx + k^2 - r^2) = 0

                    # (1 + m^2)x^2 + (2bm - 2h)x + (h^2 + b^2 - 2bk + k^2 - r^2) = 0
                    A = 1 + m*m
                    B = -2*h + 2*m*b - 2*m*k
                    # B = -2*h + 2*m*b - 2*m*k
                    C = h*h + b*b - 2*k*b + k*k - r*r


                    x3 = (-B + math.sqrt((B ** 2) - (4 * A * C))) / (2 * A)
                    x4 = (-B - math.sqrt((B ** 2) - (4 * A * C))) / (2 * A)

                    y3 = m*x3 + b
                    y4 = m*x4 + b

                    # print(x3, y3)
                    # print(x4, y4)

                    if obstacle_['alfa'] < 0:
                        pi_x = x3
                        pi_y = y3
                    else:
                        pi_x = x4
                        pi_y = y4


                    # cv2.line(self.test_image, (int(pi_x), int(pi_y)), 
                    #          (int(pi_x - self.scale * obstacle['dist'] * math.cos(math.radians(-obstacle['alfa'] + 90))),
                    #           int(pi_y - self.scale * obstacle['dist'] * math.sin(math.radians(-obstacle['alfa'] + 90)))),
                    #           (0, 255, 0))


            obstacle['length_deg'] = math.radians(obstacle['length_deg'])
            obstacle['alfa'] = -math.radians(obstacle['alfa'])
            

            # CORRECOES (pos impressoes no ambiente grafico)
            obstacle_['dist'] -= self.robot_radius
            obstacle_['length_deg'] = math.radians(obstacle_['length_deg'])
            obstacle_['alfa'] = -math.radians(obstacle_['alfa'])


            # print(obstacle_)

            obs_list_v.append(obstacle_.copy())

            obstacle_.clear()

        if self.DEBUG_PRINT:
            # print(obs_list_v)
            for obs in obs_list_v:
                pass
                print(obs)
            print()

        return obs_list_v
    
    def data_ready_to_publish(self, obs_dict_v):
        tot_obs = Obstacles()

        tot_obs.no_obstacles = len(obs_dict_v)
        # print(len(obs_dict_v))
        for o in range(len(obs_dict_v)):
            # print(obs_dict_v[o])
            # print(obs_dict_v[o]['alfa'])
            obs = ObstacleInfo()
            
            obs.alfa = obs_dict_v[o]['alfa']
            obs.dist = obs_dict_v[o]['dist']
            obs.length_cm = obs_dict_v[o]['length_cm']
            obs.length_degrees = obs_dict_v[o]['length_deg']

            tot_obs.obstacles.append(obs)
        
        # print(tot_obs)
        return tot_obs

        


class ObstaclesNode(Node):

    def __init__(self):
        super().__init__("Obstacles")
        self.get_logger().info("Initialised CHARMIE Obstacles Node")

        # Create Code Class Instance
        self.obs_detect = ObstaclesLIDAR() 

        # Create PUBs/SUBs
        self.obstacles_publisher = self.create_publisher(Obstacles, "obs_lidar", 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, "scan", self.lidar_callback , 10)
        self.obstacles_diagnostic_publisher = self.create_publisher(Bool, "obstacles_diagnostic", 10)

        flag_diagn = Bool()
        flag_diagn.data = True
        self.obstacles_diagnostic_publisher.publish(flag_diagn)

        # Create Timers
        # self.create_timer(1, self.timer_callback)
        

    def lidar_callback(self, scan:LaserScan):
        self.obs_detect.lidar_readings_to_obstacles(scan)

        if not self.obs_detect.error_lidar_reading:
            self.obstacles_publisher.publish(self.obs_detect.obstacles_pub)


    """ 
    def timer_callback(self):
        # print("Pubed Obstacles")
        a = Obstacles()

        b1 = ObstacleInfo()
        b2 = ObstacleInfo()
        b3 = ObstacleInfo()

        b1.alfa = 1.0
        b1.dist = 2.0
        b1.length_cm = 3.0
        b1.length_degrees = 4.0
        
        b2.alfa = 5.0
        b2.dist = 6.0
        b2.length_cm = 7.0
        b2.length_degrees = 8.0
        
        a.obstacles.append(b1)
        a.obstacles.append(b2)
        a.obstacles.append(b3)
        a.no_obstacles=len(a.obstacles)

        self.obstacles_publisher.publish(a)
    """



def main(args=None):
    rclpy.init(args=args)
    node = ObstaclesNode()
    rclpy.spin(node)
    rclpy.shutdown()
