#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Bool, Float32, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Pose2D, PoseWithCovarianceStamped
from charmie_interfaces.msg import TarNavSDNL, Obstacles, ListOfDetectedPerson, Obstacles
from charmie_interfaces.srv import Trigger, SetAcceleration, ActivateBool, SetRGB
from sensor_msgs.msg import Image

import cv2
import numpy as np
import math
import time
from cv_bridge import CvBridge

# Constant Variables to ease RGB_MODE coding
RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106


class NavigationSDNLClass:

    def __init__(self):

        # configurable SDNL parameters
        # self.lambda_target_mov = 12
        # self.lambda_target_rot = 12
        
        
        # self.lambda_target = 5 # 12
        # self.beta1 = 120  # 9
        # self.beta2 = 0.15 # 7

        # generic tasks
        self.lambda_target = 4
        self.lambda_target_not_obs = 10
        self.lambda_target_with_obs = 4
        self.beta1 = 40 # 40
        self.beta2 = 0.06 # 0.04
        self.min_speed_obs = 6.0 # speed
        
        # restaurant
        # self.lambda_target = 5
        # self.lambda_target_not_obs = 8
        # self.lambda_target_with_obs = 4
        # self.beta1 = 80 
        # self.beta2 = 0.35 
        # self.min_speed_obs = 3.0 # speed
        
        # configurable other parameters
        self.nav_threshold_dist = 0.6 # in meters
        # self.nav_threshold_dist_follow_me = 1.2 # in meters
        self.nav_threshold_ang = 15 # degrees
        # self.nav_threshold_ang_follow_me = 20 # degrees
        self.max_lin_speed = 15.0 # speed # 30.0
        self.max_ang_speed = 10.0 # speed # 20.0
        self.tar_dist_decrease_lin_speed = 1.2 # meters
        self.obs_dist_decrease_lin_speed = 1.5 # meters
        # self.min_speed_obs = 3.0 # speed
        self.decay_rate_initial_speed_ramp = 1.5 # seconds # time took by the initial ramp  
        self.decay_rate_initial_speed_ramp /= 0.1 # d_tao qual é feita a navigation
        self.MAX_DIST_FOR_OBS = 2.0

        self.obstacles = Obstacles()
        self.aux_obstacles_l = Obstacles()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0
        self.nav_target = TarNavSDNL()

        # IMU ORIENTATION
        self.imu_orientation = 0.0
        self.imu_orientation_norm = 0.0
        self.NORTE = 0.0
        self.first_imu_orientation = False

        self.first_nav_target = False
        self.dist_to_target = 0.0
        self.ang_to_target = 0.0 
        self.min_dist_obs = self.MAX_DIST_FOR_OBS
        self.aux_initial_speed_ramp = 0.0

        self.f_target = 0.0
        self.y_atrator = []
        self.obstacles_l = []
        self.f_obstacle = 0.0
        # self.y_repulsor1 = []
        # self.y_repulsor2 = []
        # self.y_repulsor3 = []
        # self.yf = []
        self.yff = []
        self.yfff = []
        self.f_final = 0.0
        self.y_final = []

        # PID variables:
        self.integral = 0.0
        self.prev_error = 0.0
        self.correct_ctr = 0

        # visual debug
        self.DEBUG_DRAW_IMAGE = False # debug drawing opencv
        self.xc = 400
        self.yc = 400
        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.image_plt = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.plot1 = TRplotter(133, (0, 255, 0))
        self.plot2 = TRplotter(400, (0, 255, 255))
        self.plot3 = TRplotter(800 - 133, (255, 255, 0))
        self.scale_plotter = 50 # ???
        self.scale = 0.12*1000
        self.robot_radius = 0.560/2 # meter
        self.lidar_radius = 0.050/2 # meter
        self.all_pos_x_val = []
        self.all_pos_y_val = []
        self.all_pos_t_val = []
        self.all_obs_val = []

    def sdnl_main(self, mov_or_rot):

        if mov_or_rot == "rot":
            self.nav_target.flag_not_obs = True
            self.lambda_target = self.lambda_target_not_obs 

        if mov_or_rot == "mov":
            
            if self.nav_target.flag_not_obs:
                self.lambda_target = self.lambda_target_not_obs

            else:
                self.lambda_target = self.lambda_target_with_obs
            
        print(self.max_ang_speed, self.lambda_target)

        self.f_target, self.y_atrator = self.atrator(mov_or_rot)

        #### PROXIMAS DUAS FUNCOES DEVIAM ESTAR DENTRO DO IF POR UMA QUESTAO DE EFICIENCIA
        #### POREM POR DEBUG PODEM ESTAR CA FORA
        self.obstacles_l = self.calculate_obstacles_lidar()

        # self.f_obstacle, self.y_repulsor1, self.y_repulsor2, self.y_repulsor3, self.yf, self.yff, self.yfff = self.repulsor()
        self.f_obstacle, self.yff, self.yfff = self.repulsor()
        
        if not self.nav_target.flag_not_obs:
            # self.max_lin_speed = 15.0
            self.f_final, self.y_final = self.combine_atrator_repulsor()
            # print("ATRATOR + REPULSORES")
        else:
            # in case it is intended to not consider obstacles
            # self.max_lin_speed = 20.0
            self.f_final = self.f_target
            self.y_final = self.y_atrator
            # print("SÓ ATRATOR")

        # print(self.f_final)

        self.dist_to_target = self.upload_move_dist_to_target()
        self.ang_to_target = self.upload_rot_ang_to_target()

        if self.f_final > self.max_ang_speed:
            self.f_final = self.max_ang_speed
        if self.f_final < -self.max_ang_speed:
            self.f_final = -self.max_ang_speed

        # print("Received OMNI move. dir =", omni.x, "vlin =", omni.y, "vang =", omni.z)
        omni_move = Vector3()

        if mov_or_rot == "rot":
            omni_move.y = float(0.0)
        elif mov_or_rot == "mov":
            omni_move.y = float(self.max_lin_speed)
            speed_t = self.max_lin_speed
            speed_o = self.max_lin_speed
            speed_i = self.max_lin_speed

            # decrescimo de velocidade perto do target 
            if self.dist_to_target < self.tar_dist_decrease_lin_speed:  
                lin_speed_variation = self.max_lin_speed / self.tar_dist_decrease_lin_speed
                speed_t = self.max_lin_speed - lin_speed_variation*(self.tar_dist_decrease_lin_speed - self.dist_to_target)
            
            # decrescimo de velocidade perto do obstaculo 
            if self.min_dist_obs < self.obs_dist_decrease_lin_speed:
                lin_speed_variation = (self.max_lin_speed - self.min_speed_obs) / self.obs_dist_decrease_lin_speed
                speed_o = self.max_lin_speed - lin_speed_variation*(self.obs_dist_decrease_lin_speed - self.min_dist_obs)

            # decrescimo de velocidade rampa de aceleração inicial 
            if self.aux_initial_speed_ramp <= self.max_lin_speed:
                # if self.min_dist_obs < self.obs_dist_decrease_lin_speed: # do not understand this ...
                lin_speed_variation = self.max_lin_speed / (self.decay_rate_initial_speed_ramp)
                self.aux_initial_speed_ramp += lin_speed_variation
                speed_i = self.aux_initial_speed_ramp                    

            
            if not self.nav_target.flag_not_obs:
                omni_move.y = min(speed_t, speed_o, speed_i)
                print("SPEED COM OBS", self.max_lin_speed, omni_move.y)
            else:
                omni_move.y = min(speed_t, speed_i) # remove obstacles distance reduction for cases we do not care about the obstacles
                print("SPEED SEM OBS", self.max_lin_speed, omni_move.y)

            ### DEPOIS TENHO QUE ARRANJAR MANIERA DE JUNTAR AS DUAS EQUACOES, ASSIM NAO HA SALTOS
            # print("speed_t:", speed_t, "speed_o:", speed_o, "speed_i:", speed_i, "omni_move.y:", omni_move.y)
            # print(omni_move.y)

        omni_move.z = float(100.0 - self.f_final)
        
        return omni_move

    def rotate_orientation(self, orientation_target):
        
        max_rot_speed = 10
        acceptable_error = 10
        kp = 0.14
        
        error = self.imu_orientation_norm + orientation_target

        if error > 180.0:
            error -= 360.0 
        elif error < -180:
            error += 360.0 
        
        speed = error * kp
        if speed > max_rot_speed:
            speed = max_rot_speed 
        if speed < -max_rot_speed:
            speed = -max_rot_speed 

        target_reached = False
        omni_move = Vector3()
        omni_move.x = float(0.0)
        omni_move.y = float(0.0)

        if abs(error) >= acceptable_error:
            omni_move.z = float(100.0 - speed)
            target_reached = False
        else:
            omni_move.z = float(100.0)
            target_reached = True


        print(error, speed, target_reached)

        return omni_move, target_reached

    def rotate_adjust_orientation(self, orientation_target):

        max_rot_speed = 5
        acceptable_error = 3.0
        d_t = 0.1
        # kp = 0.14
        kp = 0.08
        ki = 0.02
        kd = 0.04

        error = self.imu_orientation_norm + orientation_target
        
        if error > 180.0:
            error -= 360.0 
        elif error < -180:
            error += 360.0 
        
        P = kp * error 
        
        if error * self.prev_error < 0: # different signs
            self.integral = 0
        self.integral += error * d_t 
        I = ki * self.integral
        
        derivative = (error - self.prev_error) / d_t
        D = kd * derivative
        
        speed = P + I + D
        
        if speed > max_rot_speed:
            speed = max_rot_speed 
        if speed < -max_rot_speed:
            speed = -max_rot_speed 

        target_reached = False
        omni_move = Vector3()
        omni_move.x = float(0.0)
        omni_move.y = float(0.0)

        if abs(error) >= acceptable_error:
            omni_move.z = float(100.0 - speed)
            target_reached = False
            self.correct_ctr = 0
        else:
            omni_move.z = float(100.0)
            self.correct_ctr += 1
            
        if self.correct_ctr > 3:
            omni_move.z = float(100.0)
            target_reached = True
            self.correct_ctr = 0
            self.integral = 0

        self.prev_error = error

        print(round(error,2), round(speed,2), target_reached, self.correct_ctr, round(P,2), round(I,2), round(D,2))
        return omni_move, target_reached

    def atrator(self, mov_or_rot):

        # variables necessary to calculate the atrator
        # if mov_or_rot == "mov":
        #     target = (self.nav_target.move_target_coordinates.x, self.nav_target.move_target_coordinates.y)
        #     lambda_target = self.lambda_target_mov
        # elif mov_or_rot == "rot":
        #     target = (self.nav_target.rotate_target_coordinates.x, self.nav_target.rotate_target_coordinates.y)
        #     lambda_target = self.lambda_target_rot

        target = (self.nav_target.target_coordinates.x, self.nav_target.target_coordinates.y) 
        lambda_target = self.lambda_target
        current_pos = (self.robot_x, self.robot_y)
        phi = self.robot_t

        print("lambda_target:", lambda_target)


        phi_ = phi + math.pi/2


        psi_target = math.atan2(target[1]-current_pos[1], target[0]-current_pos[0])

        f_target = -lambda_target*math.sin(phi_ - psi_target)

        # print(math.degrees(psi_target), phi)
        # print(f_target)

        x = np.linspace(0, 2 * math.pi, 360 * 2)
        y = [0] * 360*2

        for half_degree in range(360*2):
            y[half_degree] = -lambda_target*math.sin(x[half_degree] - psi_target)
            # print(math.degrees(phi_), half_degree, x[half_degree], y[half_degree])
            # print(math.degrees(x[half_degree]), end=",")

        # print("TARGET: ", target, "CURRENT:", current_pos, "PSI_TARGET:", math.degrees(psi_target))

        # for i in range(len(y)):
        #     print(i, y[i])

        return f_target, y
    
    def calculate_obstacles_lidar(self):

        # variables necessary to calculate the obstacles from the LIDAR
        phi = self.robot_t

        obstacles = []
        aux_obs = {}
        phi_ =  phi + math.pi/2

        min_d = self.MAX_DIST_FOR_OBS
        for obs in self.obstacles.obstacles:
            if obs.dist < self.MAX_DIST_FOR_OBS:
                if obs.dist < min_d:
                    min_d = obs.dist
                # print(obs)
                aux_obs['psi_obs'] = phi_ - obs.alfa
                aux_obs['dis_obs'] = obs.dist
                aux_obs['del_tet'] = obs.length_angular

                obstacles.append(aux_obs.copy())

        self.min_dist_obs = min_d
        # print()
        return obstacles

    def repulsor(self):

        obstacles = self.obstacles_l
        phi = self.robot_t

        # variables to help in the TRplotter
        y1 = [0] * 360 * 2
        y2 = [0] * 360 * 2
        y3 = [0] * 360 * 2
        yf = [0] * 360 * 2
        h = [0] * 3
        yff = []
        yfff = [0] * 360 * 2

        f_obstacle = 0
        f_obs = 0
        # robot_radius = 560/2  # mm ## TENHO DE ALTERAR PARA DE mm para m


        for obs in obstacles:

            # print(obs, end='\t')
            psi_obstacle = obs['psi_obs']  # + math.pi/2
            d_obs = obs['dis_obs']
            delta_teta = obs['del_tet']

            y1 = [0] * 360 * 2
            y2 = [0] * 360 * 2
            y3 = [0] * 360 * 2
            yf = [0] * 360 * 2
            h = [0] * 3

            # sigma = 1
            # sigma_ = 2*math.atan(math.tan(delta_teta/2) + robot_radius/(robot_radius+d_obs))

            

            lambda_obstacle_ = self.beta1*math.exp(-d_obs/self.beta2)
            sigma_ = math.atan(math.tan(delta_teta/2) + self.robot_radius/(self.robot_radius+d_obs))
            # print(lambda_obstacle_)

            # sigma_ /= 1.3


            ### phi_ = phi + 90
            ### if phi_ > 360:
            ###     phi_ -= 360
            phi_ = phi + math.pi/2

            # print(round(math.degrees(phi_), 2), int(phi_/(2*math.pi)), end=', ')

            if phi_ > 0:
                qi = int(phi_/(2*math.pi)) * (2*math.pi)
                qf = int(phi_/(2*math.pi)) * (2*math.pi) + (2*math.pi)
            else:
                qi = int(phi_ / (2 * math.pi)) * (2 * math.pi) - (2 * math.pi)
                qf = int(phi_ / (2 * math.pi)) * (2 * math.pi)

            # print(math.degrees(qi), math.degrees(qf))

            x1 = np.linspace(qi - 2 * math.pi, qi, 360 * 2)
            x2 = np.linspace(qi, qf, 360 * 2)
            x3 = np.linspace(qf, qf + 2 * math.pi, 360 * 2)


            # print("LAMBDA: ", lambda_obstacle_, "SIGMA: ", sigma_)
            ### f_target = -lambda_target*math.sin(phi_ - psi_target)

            f_obs = lambda_obstacle_ * (phi_ - psi_obstacle) * math.exp(-((phi_ - psi_obstacle) ** 2) / (2 * (sigma_ ** 2)))
            f_obstacle += f_obs
            # print(f_obs)

            # print(round(d_obs,2), round(delta_teta,2), round(lambda_obstacle_,2), round(sigma_,2), round(f_obs,2))

            for half_degree in range(360*2):

                # y[half_degree] = -lambda_target*math.sin(x[half_degree] - psi_target)

                y1[half_degree] = lambda_obstacle_ * (x1[half_degree] - psi_obstacle) * \
                    math.exp(-((x1[half_degree] - psi_obstacle)**2) / (2 * sigma_ ** 2))

                y2[half_degree] = lambda_obstacle_ * (x2[half_degree] - psi_obstacle) * \
                    math.exp(-((x2[half_degree] - psi_obstacle)**2) / (2 * sigma_ ** 2))

                y3[half_degree] = lambda_obstacle_ * (x3[half_degree] - psi_obstacle) * \
                    math.exp(-((x3[half_degree] - psi_obstacle)**2) / (2 * sigma_ ** 2))

                h[0] = y1[half_degree]
                h[1] = y2[half_degree]
                h[2] = y3[half_degree]

                yf[half_degree] = max(h, key=abs)  # finds maximum absolute value of list

                yfff[half_degree] += yf[half_degree]

            # print(yf)
            yff.append(yf)
            # yfff += yff
            # print(y)

        # print(f_obstacle)

        # print("TARGET: ", target, "CURRENT:", current_pos, "PSI_TARGET:", math.degrees(psi_target))
        # print(yff)
        # return f_obstacle, y1, y2, y3, yf, yff, yfff
        return f_obstacle, yff, yfff

    def combine_atrator_repulsor(self):

        mu = 0
        sigma = 1

        f_stoch = np.random.normal(mu, sigma, size=1)  # 1000 samples with normal distribution
        # print(f_stoch)

        ffinal = self.f_target + self.f_obstacle + f_stoch
        # y_final = y_atrator+yf_repulsor

        y_final = [0] * 360 * 2

        for t in range(360*2):
            y_final[t] = self.y_atrator[t] + self.yfff[t]

        return ffinal, y_final

    def upload_move_dist_to_target(self):
        
        x1 = self.robot_x
        y1 = self.robot_y

        # x2 = self.nav_target.move_target_coordinates.x
        # y2 = self.nav_target.move_target_coordinates.y
        
        x2 = self.nav_target.target_coordinates.x
        y2 = self.nav_target.target_coordinates.y

        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)

        return dist

    def upload_rot_ang_to_target(self):

        # target = (self.nav_target.rotate_target_coordinates.x, self.nav_target.rotate_target_coordinates.y)
        
        target = (self.nav_target.target_coordinates.x, self.nav_target.target_coordinates.y)
        current_pos = (self.robot_x, self.robot_y)

        # theta1 = math.degrees(self.robot_t + math.pi/2)
        # theta2 = math.degrees(math.atan2(target[1]-current_pos[1], target[0]-current_pos[0]))
        theta1 = (self.robot_t + math.pi/2)
        theta2 = (math.atan2(target[1]-current_pos[1], target[0]-current_pos[0]))

        # normalize_angles()
        
        # print("THETA2", theta2)
        # ang = abs(theta1 - theta2)
        # print("ang1:", ang)

        # ang = abs(math.degrees(theta1 - theta2))
        # print("ang2:", ang)

        ang = abs(self.normalize_angles(math.degrees(theta1 - theta2)))
        # print("ang3:", ang)
        
        return ang

    def normalize_angles(self, ang):

        while ang < -180:
            ang += 360
        while ang >= 180:
            ang -= 360

        return ang

    def update_debug_drawings(self):
            
        if self.DEBUG_DRAW_IMAGE:

            # 1 meter lines horizontal and vertical
            for i in range(20):
                cv2.line(self.test_image, (int(self.xc + self.scale*i), 0), (int(self.xc + self.scale*i), self.xc*2), (255, 255, 255), 1)
                cv2.line(self.test_image, (int(self.xc - self.scale*i), 0), (int(self.xc - self.scale*i), self.xc*2), (255, 255, 255), 1)
                cv2.line(self.test_image, (0, int(self.yc - self.scale*i)), (self.yc*2, int(self.yc - self.scale*i)), (255, 255, 255), 1)
                cv2.line(self.test_image, (0, int(self.yc + self.scale*i)), (self.yc*2, int(self.yc + self.scale*i)), (255, 255, 255), 1)
            
            # present and past localization positions
            self.all_pos_x_val.append(self.robot_x)
            self.all_pos_y_val.append(self.robot_y)
            self.all_pos_t_val.append(self.robot_t)
            for i in range(len(self.all_pos_x_val)):
                cv2.circle(self.test_image, (int(self.xc + self.scale*self.all_pos_x_val[i]), int(self.yc - self.scale * self.all_pos_y_val[i])), 1, (255, 255, 0), -1)

            # obstacles
            self.all_obs_val.append(self.aux_obstacles_l)
            
            """
            # past obstacles
            for j in range(len(self.all_obs_val)):

                for i in range(self.all_obs_val[j].no_obstacles):

                    # aux variables
                    aux_ang = self.all_obs_val[j].obstacles[i].alfa
                    aux_dist = self.all_obs_val[j].obstacles[i].dist + self.robot_radius
                    aux_new_obs_len = self.all_obs_val[j].obstacles[i].dist * math.tan(self.all_obs_val[j].obstacles[i].length_angular)
                    thickness = 20

                    # line length of obstacle at dist and alfa detected
                    cv2.line(self.test_image, (int(self.xc + self.scale*self.all_pos_x_val[j] - self.scale * (aux_dist) * (math.cos(aux_ang - self.all_pos_t_val[j] + math.pi/2)) + self.scale * (aux_new_obs_len/2) * math.cos(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2))),
                                               int(self.yc - self.scale*self.all_pos_y_val[j] - self.scale * (aux_dist) * (math.sin(aux_ang - self.all_pos_t_val[j] + math.pi/2)) - self.scale * (aux_new_obs_len/2) * math.sin(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2)))),
                                              (int(self.xc + self.scale*self.all_pos_x_val[j] - self.scale * (aux_dist) * (math.cos(aux_ang - self.all_pos_t_val[j] + math.pi/2)) - self.scale * (aux_new_obs_len/2) * math.cos(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2))),
                                               int(self.yc - self.scale*self.all_pos_y_val[j] - self.scale * (aux_dist) * (math.sin(aux_ang - self.all_pos_t_val[j] + math.pi/2)) + self.scale * (aux_new_obs_len/2) * math.sin(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2)))),
                                              (0, 165, 255), int(1.0 + thickness*self.scale/1000))
            """

            thickness = 20
            # current obstacles
            for i in range(self.aux_obstacles_l.no_obstacles):

                # aux variables
                aux_ang = self.aux_obstacles_l.obstacles[i].alfa
                aux_dist = self.aux_obstacles_l.obstacles[i].dist + self.robot_radius
                aux_new_obs_len = self.aux_obstacles_l.obstacles[i].dist * math.tan(self.aux_obstacles_l.obstacles[i].length_angular)

                #line robot center to obstacle center
                # cv2.line(self.test_image, (int(self.xc + self.scale*self.robot_x), int(self.yc - self.scale * self.robot_y)),
                #                         (int(self.xc + self.scale*self.robot_x - self.scale * (aux_dist) * (math.cos(aux_ang - self.robot_t + math.pi/2))),
                #                         int(self.yc - self.scale*self.robot_y - self.scale * (aux_dist) * (math.sin(aux_ang - self.robot_t + math.pi/2)))),
                #                         (255, 255, 255))

                # line robot platform to obstacle center
                cv2.line(self.test_image, (int(self.xc + self.scale*self.robot_x - self.scale * (self.robot_radius) * (math.cos(aux_ang - self.robot_t + math.pi/2))),
                                           int(self.yc - self.scale*self.robot_y - self.scale * (self.robot_radius) * (math.sin(aux_ang - self.robot_t + math.pi/2)))),
                                          (int(self.xc + self.scale*self.robot_x - self.scale * (aux_dist) * (math.cos(aux_ang - self.robot_t + math.pi/2))),
                                           int(self.yc - self.scale*self.robot_y - self.scale * (aux_dist) * (math.sin(aux_ang - self.robot_t + math.pi/2)))),
                                          (255, 255, 255))
                
                # line length of obstacle at dist and alfa detected
                cv2.line(self.test_image, (int(self.xc + self.scale*self.robot_x - self.scale * (aux_dist) * (math.cos(aux_ang - self.robot_t + math.pi/2)) + self.scale * (aux_new_obs_len/2) * math.cos(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2))),
                                        int(self.yc - self.scale*self.robot_y - self.scale * (aux_dist) * (math.sin(aux_ang - self.robot_t + math.pi/2)) - self.scale * (aux_new_obs_len/2) * math.sin(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2)))),
                                        (int(self.xc + self.scale*self.robot_x - self.scale * (aux_dist) * (math.cos(aux_ang - self.robot_t + math.pi/2)) - self.scale * (aux_new_obs_len/2) * math.cos(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2))),
                                        int(self.yc - self.scale*self.robot_y - self.scale * (aux_dist) * (math.sin(aux_ang - self.robot_t + math.pi/2)) + self.scale * (aux_new_obs_len/2) * math.sin(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2)))),
                                        (0, 0, 255), int(1.0 + thickness*self.scale/1000))
                  
            
            # targets
            if self.first_nav_target:
                """
                cv2.circle(self.test_image, (int(self.xc + self.scale*self.nav_target.move_target_coordinates.x), int(self.yc - self.scale*self.nav_target.move_target_coordinates.y)), 
                                            (int)(self.scale*self.robot_radius/2), (0, 255, 0), -1)
                cv2.circle(self.test_image, (int(self.xc + self.scale*self.nav_target.rotate_target_coordinates.x), int(self.yc - self.scale*self.nav_target.rotate_target_coordinates.y)), 
                                            (int)(self.scale*self.robot_radius/2), (0, 150, 0), -1)  


                aux_ang_tar = math.atan2(self.nav_target.move_target_coordinates.y - self.nav_target.rotate_target_coordinates.y, self.nav_target.move_target_coordinates.x - self.nav_target.rotate_target_coordinates.x)
                cv2.line(self.test_image,   (int(self.xc + self.scale*self.nav_target.move_target_coordinates.x), 
                                             int(self.yc - self.scale*self.nav_target.move_target_coordinates.y)),
                                            (int(self.xc + self.scale*self.nav_target.move_target_coordinates.x - self.scale * self.robot_radius * math.cos(aux_ang_tar)),# + math.pi/2)), 
                                             int(self.yc - self.scale*self.nav_target.move_target_coordinates.y + self.scale * self.robot_radius * math.sin(aux_ang_tar))),# + math.pi/2))),
                                            (0, 255, 0), int(1.0 + thickness*self.scale/1000))
                """         
            
                cv2.circle(self.test_image, (int(self.xc + self.scale*self.nav_target.target_coordinates.x), int(self.yc - self.scale*self.nav_target.target_coordinates.y)), 
                                            (int)(self.scale*self.robot_radius/2), (0, 255, 0), -1)
                # cv2.circle(self.test_image, (int(self.xc + self.scale*self.nav_target.target_coordinates.x), int(self.yc - self.scale*self.nav_target.target_coordinates.y)), 
                #                             (int)(self.scale*self.robot_radius/2), (0, 150, 0), -1)  


                # aux_ang_tar = math.atan2(self.nav_target.target_coordinates.y - self.nav_target.target_coordinates.y, self.nav_target.target_coordinates.x - self.nav_target.target_coordinates.x)
                # cv2.line(self.test_image,   (int(self.xc + self.scale*self.nav_target.target_coordinates.x), 
                #                              int(self.yc - self.scale*self.nav_target.target_coordinates.y)),
                #                             (int(self.xc + self.scale*self.nav_target.target_coordinates.x - self.scale * self.robot_radius * math.cos(aux_ang_tar)),# + math.pi/2)), 
                #                              int(self.yc - self.scale*self.nav_target.target_coordinates.y + self.scale * self.robot_radius * math.sin(aux_ang_tar))),# + math.pi/2))),
                #                             (0, 255, 0), int(1.0 + thickness*self.scale/1000))
                


            # robot
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x), int(self.yc - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x), int(self.yc - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius/10), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                            int(self.yc - self.scale*self.robot_y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius), (0, 255, 255), -1)
            


            # THIS IF HAS TO BE CHANGED TO A IF AFTER CALCULATING THE VALUES FOR THE DRAW: 
            if self.first_nav_target:
                # SDNL equations
                # if self.first_nav_target:
                # print(self.y_atrator)
                self.plot1.plot(self.image_plt, self.scale_plotter, self.y_atrator, self.robot_t)
                for y_plt_ff in self.yff:
                    self.plot3.plot(self.image_plt, self.scale_plotter, y_plt_ff, self.robot_t, (255, 100, 100))

                # print(self.yfff)
                # print(self.y_final)
                    
                self.plot2.plot(self.image_plt, self.scale_plotter, self.yfff, self.robot_t, (0, 140, 255))
                self.plot2.plot(self.image_plt, self.scale_plotter, self.y_final, self.robot_t, (255, 255, 0))
                
                cv2.imshow("SDNL", self.image_plt)


            cv2.imshow("Navigation SDNL", self.test_image)
            
            k = cv2.waitKey(1)
            if k == ord('+'):
                self.scale /= 0.8
            if k == ord('-'):
                self.scale *= 0.8
            if k == ord('0'):
                self.all_pos_x_val.clear()
                self.all_pos_y_val.clear()

            self.test_image[:, :] = 0
            self.image_plt[:, :] = 0

    def obstacles_msg_to_position(self, obs: Obstacles):
        self.obstacles = obs
        # self.aux_obstacles_l = obs

        aux_index = []
        aux_obs_l = Obstacles()
        """
        for i in range(len(self.aux_obstacles_l.obstacles)):
            if self.aux_obstacles_l.obstacles[i].dist > self.MAX_DIST_FOR_OBS:
                print(self.aux_obstacles_l.obstacles[i].dist)
                # print(type(self.aux_obstacles_l.obstacles))
                print(i)
                aux_index.append(i)
                
        
        for j in aux_index[::-1]:
            print(j)

            self.aux_obstacles_l.obstacles.remove(j)

        """

        for obs_ in self.obstacles.obstacles:
            if obs_.dist < self.MAX_DIST_FOR_OBS:
                aux_obs_l.obstacles.append(obs_)

        aux_obs_l.no_obstacles = len(aux_obs_l.obstacles)

        self.aux_obstacles_l = aux_obs_l

        # print(self.aux_obstacles_l)
        # print("###")



        # print(self.obstacles)

    def navigation_msg_to_position(self, nav: TarNavSDNL):
        self.nav_target = nav
        # knows when the first targets are sent, for visual debug
        if not self.first_nav_target:
            self.first_nav_target = True
        # print(self.nav_target)


class TRplotter:

    def __init__(self, coord_y, colour=(255, 255, 255)):

        self.coord_y = coord_y
        self.colour = colour
        self.xc = 400
        self.yc = 400
        self.x = np.linspace(0, 2 * math.pi, 360 * 2)
        self.centre_data = int((self.xc*2 - len(self.x)) / 2)

    def plot(self, image_plt, scale, y, theta, plt_colour=None):

        # if theta > 360*2:
        #     theta -= 360*2

        if plt_colour is None:
            fcolour = self.colour
        else:
            fcolour = plt_colour

        cv2.line(image_plt, (image_plt.shape[1] - self.centre_data - 0, self.coord_y),
                 (image_plt.shape[1] - self.centre_data - len(self.x), self.coord_y), (255, 255, 255))
        # esq
        cv2.line(image_plt, (image_plt.shape[1] - self.centre_data - len(self.x), self.coord_y - int(2 * scale)),
                 (image_plt.shape[1] - self.centre_data - len(self.x), self.coord_y + int(2 * scale)), (255, 255, 255))
        # dir
        cv2.line(image_plt, (image_plt.shape[1] - self.centre_data - 0, self.coord_y - int(2 * scale)),
                 (image_plt.shape[1] - self.centre_data - 0, self.coord_y + int(2 * scale)), (255, 255, 255))
        # top
        cv2.line(image_plt, (image_plt.shape[1] - self.centre_data - 0, self.coord_y - int(2 * scale)),
                 (image_plt.shape[1] - self.centre_data - len(self.x), self.coord_y - int(2 * scale)), (255, 255, 255))
        # bottom
        cv2.line(image_plt, (image_plt.shape[1] - self.centre_data - 0, self.coord_y + int(2 * scale)),
                 (image_plt.shape[1] - self.centre_data - len(self.x), self.coord_y + int(2 * scale)), (255, 255, 255))

        # print("x = ", len(self.x))

        # print("y = ", y)

        if y: # checks if list is not empty

            for value in range(len(self.x)):
                # print(value, end="")
                # cv2.circle(image_plt, (image_plt.shape[1] - self.centre_data - value, int(self.coord_y - scale/10 * y[value])), 0, (100,100,100), 0)
                # print(value)
                cv2.circle(image_plt, (self.centre_data + value, int(self.coord_y - scale/20 * y[value])), 0, fcolour, 0)
                
                pass
                # cv2.line(image_plt, (image_plt.shape[1] - centre_data - value, yc - 200),
                #          (image_plt.shape[1] - centre_data - value, int(yc - 200 - scale * y[value])),
                #          (255, 0, 0))

        # theta on the three plots
        # print(theta)
        ### cv2.line(image_plt, (self.centre_data + theta, self.coord_y - int(2 * scale)),
        ###         (self.centre_data + theta, self.coord_y + int(2 * scale)), (0, 0, 255))

        theta += math.pi/2

        # top
        # cv2.line(image_plt, (image_plt.shape[1] - self.centre_data - 0, int(self.coord_y - scale/20 * 17.9)),
        #          (image_plt.shape[1] - self.centre_data - len(self.x), int(self.coord_y - scale/20 * 17.9)), (255, 255, 255))


        cv2.line(image_plt, (self.centre_data + self.normalize_angles(int(math.degrees(theta)))*2, self.coord_y - int(2 * scale)),
                (self.centre_data + self.normalize_angles(int(math.degrees(theta)))*2, self.coord_y + int(2 * scale)), (0, 0, 255))

        # print(self.normalize_angles(int(math.degrees(theta))))

    def normalize_angles(self, ang):

        while ang < 0:
            ang += 360
        while ang >= 360:
            ang -= 360

        return ang

        
class NavSDNLNode(Node):

    def __init__(self):
        super().__init__("NavigationSDNL")
        self.get_logger().info("Initialised CHARMIE NavigationSDNL Node")

        # Create Code Class Instance
        self.nav = NavigationSDNLClass() 

        ### TOPICS ###
        # Low Level 
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
        # Create PUBs/SUBs
        self.obs_lidar_subscriber = self.create_subscription(Obstacles, "obs_lidar", self.obs_lidar_callback, 10)
        # Robot Localisation
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)
        self.initialpose_subscriber = self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.get_initialpose_callback, 10)
        # Orientation
        self.get_orientation_subscribrer = self.create_subscription(Float32, "get_orientation", self.get_orientation_callback, 10)
        # Navigation
        self.target_pos_subscriber = self.create_subscription(TarNavSDNL, "target_pos", self.target_pos_callback, 10)
        self.target_pos_check_answer_publisher = self.create_publisher(Bool, "target_pos_check_answer", 10)
        self.flag_pos_reached_publisher = self.create_publisher(Bool, "flag_pos_reached", 10) 
        # Camera
        self.aligned_depth_image_subscriber = self.create_subscription(Image, "/CHARMIE/D455_head/aligned_depth_to_color/image_raw", self.get_aligned_depth_image_callback, 10)
        # Yolo Pose
        self.person_pose_filtered_subscriber = self.create_subscription(ListOfDetectedPerson, "person_pose_filtered", self.person_pose_filtered_callback, 10)

        ### SERVICES ###
        # Low level
        self.activate_orientation_client = self.create_client(ActivateBool, "activate_orientation")
        self.set_acceleration_ramp_client = self.create_client(SetAcceleration, "set_acceleration_ramp")
        self.set_rgb_client = self.create_client(SetRGB, "rgb_mode")

        while not self.set_acceleration_ramp_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Low Level...")

        self.create_timer(0.1, self.timer_callback)

        self.br = CvBridge()
        self.depth_img = Image()
        self.first_depth_image_received = False

        self.navigation_state = 0
        self.obstacles = Obstacles()
        # self.MIN_DIST_OBS = 10.0
        self.detected_people = ListOfDetectedPerson()
        self.PERSON_IN_FRONT = False
        self.latest_localisation_x = 0.0
        self.latest_localisation_y = 0.0
        self.latest_localisation_t = 0.0

        self.rgb_success = True
        self.rgb_message = ""

        request = ActivateBool.Request()
        request.activate = True
        self.activate_orientation_client.call_async(request)

        self.create_service(Trigger, 'nav_trigger', self.navigation_trigger_callback)

        # ori = Bool()
        # ori.data = True
        # self.flag_orientation_publisher.publish(ori) 


    # def obstacles_callback(self, obs:Obstacles):
    #     self.obstacles = obs
    #         
    #     self.MIN_DIST_OBS = 10.0
    # 
    #     # if len(self.obstacles.obstacles) == 0:
    #     #     print("NO OBSTACLES")
    #     # else:
    #     for obs in self.obstacles.obstacles:
    #         if obs.dist < self.MIN_DIST_OBS:
    #             self.MIN_DIST_OBS = obs.dist
    # 
    #     # print(self.MIN_DIST_OBS)        

    def get_aligned_depth_image_callback(self, img: Image):
        self.depth_img = img
        self.first_depth_image_received = True
        print("Received Depth Image")

    def person_pose_filtered_callback(self, det_people: ListOfDetectedPerson):
        self.detected_people = det_people

        print(self.PERSON_IN_FRONT)
        
        if len(self.detected_people.persons) > 0 or self.check_person_depth_head():
            self.PERSON_IN_FRONT = True
            
            if self.nav.nav_target.avoid_people:
                omni_move = Vector3()
                omni_move.x = float(0.0)
                omni_move.y = float(0.0)
                omni_move.z = float(100.0)
                self.omni_move_publisher.publish(omni_move)
                self.set_rgb(RED+SET_COLOUR)

        else:
            self.PERSON_IN_FRONT = False

        # current_frame = self.br.imgmsg_to_cv2(self.detected_people.image_rgb, "bgr8")
        # current_frame_draw = current_frame.copy()
        # cv2.imshow("Yolo Pose TR Detection 2", current_frame_draw)
        # cv2.waitKey(10)

    def set_rgb(self, command=0, wait_for_end_of=True):

        request = SetRGB.Request()
        request.colour = int(command)

        self.set_rgb_client.call_async(request)
        self.rgb_success = True
        self.rgb_message = "Value Sucessfully Sent"

        return self.rgb_success, self.rgb_message

    ### ACTIVATE OBSTACLES SERVER FUNCTIONS ###
    def call_set_acceleration_ramp_server(self, acceleration):
        request = SetAcceleration.Request()
        request.data = acceleration
        self.set_acceleration_ramp_client.call_async(request)


    def navigation_trigger_callback(self, request, response): # this only exists to have a service where we can: "while not self.nav_trigger_client.wait_for_service(1.0):"
        # Type of service received: 
        # (nothing)
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages
        
        response.success = True
        response.message = "Nav Trigger"
        return response


    def obs_lidar_callback(self, obs: Obstacles):
        # updates the obstacles variable
        self.nav.obstacles_msg_to_position(obs)
        # self.nav.sdnl_main()
        # self.nav.update_debug_drawings()
        # print("here")

        # print("===")
        # for o in obs.obstacles:
        #     print("alfa", o.alfa)
        #     print("dist", o.dist)
        #     print("l_ang", o.length_angular)
        # print("---")

        # self.MIN_DIST_OBS = 10.0
        # for o in obs.obstacles:
        #     if o.dist < self.MIN_DIST_OBS:
        #         self.MIN_DIST_OBS = o.dist

    def robot_localisation_callback(self, pose: Pose2D):
        self.nav.robot_x = pose.x
        self.nav.robot_y = pose.y
        self.nav.robot_t = pose.theta
        self.nav.update_debug_drawings()

    def get_orientation_callback(self, orientation: Float32):
        # print("Received Orientatation:", str(orientation.data))
        self.nav.imu_orientation = orientation.data
        self.nav.imu_orientation_norm = self.nav.imu_orientation - self.nav.NORTE
        if self.nav.imu_orientation_norm > 180.0:
            self.nav.imu_orientation_norm -= 360.0
        if self.nav.imu_orientation_norm < -180.0:
            self.nav.imu_orientation_norm += 360.0

        # just a flag to know for sure that to calculate NORTH we already have an IMU reading
        if self.nav.first_imu_orientation == False:
            self.nav.first_imu_orientation = True


    def get_initialpose_callback(self, pose:PoseWithCovarianceStamped):
        
        # self.initialpose.x = pose.pose.pose.position.x
        # self.initialpose.y = pose.pose.pose.position.y
        
        qx = pose.pose.pose.orientation.x
        qy = pose.pose.pose.orientation.y
        qz = pose.pose.pose.orientation.z
        qw = pose.pose.pose.orientation.w

        self.get_logger().info("RECEIVED INITIALPOSE")

        # yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        # pitch = math.asin(-2.0*(qx*qz - qw*qy))
        # roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        # print(yaw, pitch, roll)
        if self.nav.first_imu_orientation:
            self.nav.NORTE = self.nav.imu_orientation + math.degrees(math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)) 
            if self.nav.NORTE >= 360.0:
                self.nav.NORTE -= 360.0
            if self.nav.NORTE < 0.0:
                self.nav.NORTE += 360.0
        else:
            self.get_logger().error("NO IMU ORIENTATION READING, PLEASE CHECK IF LOW LEVEL WAS INITIALISED BEFORE NAVIGATION")

    def target_pos_callback(self, nav: TarNavSDNL):
        # calculates the velocities and sends them to the motors considering the latest obstacles and odometry position
        # print(nav)
        self.nav.navigation_msg_to_position(nav)
        self.nav.dist_to_target = self.nav.upload_move_dist_to_target()
        self.nav.ang_to_target = self.nav.upload_rot_ang_to_target()
        self.nav.nav_threshold_dist = nav.reached_radius # in meters
        self.nav.max_lin_speed = nav.max_speed
        self.nav.aux_initial_speed_ramp = 0.0
        self.latest_localisation_x = self.nav.robot_x
        self.latest_localisation_y = self.nav.robot_y
        self.latest_localisation_t = self.nav.robot_t
        self.navigation_state = 0

        # if self.nav.nav_target.move_or_rotate.lower() == "adjust_angle":
        #     self.call_set_acceleration_ramp_server(10)
            # send speed ramp = min

        if self.nav.nav_target.move_or_rotate.lower() == "move":
            self.call_set_acceleration_ramp_server(10)

        # since we can not use services (actually we should be using actions)
        # because it gets the code stuck if we need to adjust the navigation information during a navigation
        # we use a topic to provide confirmation that the navigation was sucessfully received.
        # On the "other side" should be a checking system that the confirmation was received and resent if not. 
        temp = Bool()
        temp.data = True
        self.target_pos_check_answer_publisher.publish(temp)
        
        # time.sleep(1.0)


    def check_person_depth_head(self, half_image_zero_or_near_percentage=0.6, full_image_near_percentage=0.3, near_max_dist=800):

        overall = False
        DEBUG = False

        print(self.first_depth_image_received)

        if self.first_depth_image_received:

            print("in")
            current_frame_depth_head = self.br.imgmsg_to_cv2(self.depth_img, desired_encoding="passthrough")
            height, width = current_frame_depth_head.shape
            current_frame_depth_head_half = current_frame_depth_head[height//2:height,:]
            
            # FOR THE FULL IMAGE

            tot_pixeis = height*width 
            mask_zero = (current_frame_depth_head == 0)
            mask_near = (current_frame_depth_head > 0) & (current_frame_depth_head <= near_max_dist)
            
            if DEBUG:
                mask_remaining = (current_frame_depth_head > near_max_dist) # just for debug
                blank_image = np.zeros((height,width,3), np.uint8)
                blank_image[mask_zero] = [255,255,255]
                blank_image[mask_near] = [255,0,0]
                blank_image[mask_remaining] = [0,0,255]

            pixel_count_zeros = np.count_nonzero(mask_zero)
            pixel_count_near = np.count_nonzero(mask_near)

            # FOR THE BOTTOM HALF OF THE IMAGE

            mask_zero_half = (current_frame_depth_head_half == 0)
            mask_near_half = (current_frame_depth_head_half > 0) & (current_frame_depth_head_half <= near_max_dist)
            
            if DEBUG:
                mask_remaining_half = (current_frame_depth_head_half > near_max_dist) # just for debug
                blank_image_half = np.zeros((height//2,width,3), np.uint8)
                blank_image_half[mask_zero_half] = [255,255,255]
                blank_image_half[mask_near_half] = [255,0,0]
                blank_image_half[mask_remaining_half] = [0,0,255]
                    
            pixel_count_zeros_half = np.count_nonzero(mask_zero_half)
            pixel_count_near_half = np.count_nonzero(mask_near_half)
            
            if DEBUG:
                cv2.line(blank_image, (0, height//2), (width, height//2), (0,0,0), 3)
                cv2.imshow("New Img Distance Inspection", blank_image)
                cv2.waitKey(10)

            half_image_zero_or_near = False
            half_image_zero_or_near_err = 0.0
            
            full_image_near = False
            full_image_near_err = 0.0


            half_image_zero_or_near_err = (pixel_count_zeros_half+pixel_count_near_half)/(tot_pixeis//2)
            if half_image_zero_or_near_err >= half_image_zero_or_near_percentage:
                half_image_zero_or_near = True
            
            full_image_near_err = pixel_count_near/tot_pixeis
            if full_image_near_err >= full_image_near_percentage:
                full_image_near = True
            
            
            if half_image_zero_or_near or full_image_near:
                overall = True

            # just for debug
            # print(overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err)
            # return overall, half_image_zero_or_near, half_image_zero_or_near_err, full_image_near, full_image_near_err
        
        return overall

    
    def timer_callback(self):

        # safety
        # if self.nav.first_imu_orientation == False:
        #     self.get_logger().warning("HAD TO RESEND IMU REQUEST...")
        #     ori = Bool()
        #     ori.data = True
        #     self.flag_orientation_publisher.publish(ori) 

        # print(self.nav.first_imu_orientation, round(self.nav.NORTE, 2), round(self.nav.imu_orientation, 2), round(self.nav.imu_orientation_norm, 2))
        
        if self.nav.first_nav_target:

            
            print("lambda_target:", self.nav.lambda_target, "beta1:", self.nav.beta1, "beta2:", self.nav.beta2)

            # print("estado:", self.navigation_state)
            
            if self.navigation_state == 0:

                if self.nav.nav_target.move_or_rotate.lower() == "rotate":

                    self.nav.max_ang_speed = 20.0 # speed # 20.0
                    self.nav.lambda_target = 12 # 12
                    omni_move = self.nav.sdnl_main("rot")
                    self.omni_move_publisher.publish(omni_move)
                    # print("DIST_ERR:", self.nav.dist_to_target)
                    # print("ANG_ERR:", self.nav.ang_to_target) 
                    if self.nav.ang_to_target <= self.nav.nav_threshold_ang:
                        self.navigation_state = 2


                elif self.nav.nav_target.move_or_rotate.lower() == "move":
        
                    self.nav.max_ang_speed = 10.0 # speed # 20.0
                    self.nav.lambda_target = 5 # 12
                    
                    if self.nav.nav_target.avoid_people and self.PERSON_IN_FRONT:
                        self.set_rgb(RED+SET_COLOUR)
                        omni_move = Vector3()
                        omni_move.x = float(0.0)
                        omni_move.y = float(0.0)
                        omni_move.z = float(100.0) 
                    else:
                        if self.nav.nav_target.avoid_people:
                            self.set_rgb(BLUE+HALF_ROTATE)
                        omni_move = self.nav.sdnl_main("mov")

                        # TESTAR: ADICIONAR TAMBEM QUANDO RECEBO YOLO POSE ??? 

                    self.omni_move_publisher.publish(omni_move)
                    # print("DIST_ERR:", self.nav.dist_to_target)
                    # print("ANG_ERR:", self.nav.ang_to_target) 
                    if self.nav.dist_to_target <= self.nav.nav_threshold_dist:
                        self.navigation_state = 2
                

                elif self.nav.nav_target.move_or_rotate.lower() == "orientate":

                    omni_move, target_reached = self.nav.rotate_orientation(self.nav.nav_target.orientation_absolute)
                    self.omni_move_publisher.publish(omni_move)
                    if target_reached:
                        self.navigation_state = 2

                elif self.nav.nav_target.move_or_rotate.lower() == "adjust_angle":

                    omni_move, target_reached = self.nav.rotate_adjust_orientation(self.nav.nav_target.orientation_absolute)
                    self.omni_move_publisher.publish(omni_move)
                    if target_reached:
                        self.navigation_state = 2

                elif self.nav.nav_target.move_or_rotate.lower() == "adjust":

                    print("Entrei adjust NORMAL")
                    print("Adjust params:", self.nav.nav_target.adjust_distance, self.nav.nav_target.adjust_direction, self.nav.nav_target.adjust_min_dist)
                    print("Latest Localisation:", self.latest_localisation_x, self.latest_localisation_y, self.latest_localisation_t)
            
                    omni_move = Vector3()
                    omni_move.x = float(self.nav.nav_target.adjust_direction)
                    omni_move.y = float(5.0)
                    omni_move.z = float(100.0) 
                    
                    dist = math.dist((self.latest_localisation_x, self.latest_localisation_y),(self.nav.robot_x, self.nav.robot_y))
                    if dist >= self.nav.nav_target.adjust_distance - 0.08: # this values is used to compensate the error made by the decceleration ramp by the motor controller
                        omni_move.x = float(0.0)
                        omni_move.y = float(0.0)
                        omni_move.z = float(100.0)
                        self.navigation_state = 2

                    self.omni_move_publisher.publish(omni_move)


                elif self.nav.nav_target.move_or_rotate.lower() == "adjust_obstacle":

                    print("Entrei adjust OBSTACLE")
                    print(self.nav.nav_target.adjust_direction, self.nav.nav_target.adjust_min_dist)
                    self.nav.calculate_obstacles_lidar()

                    omni_move = Vector3()
                    omni_move.x = float(self.nav.nav_target.adjust_direction)
                    omni_move.y = float(5.0)
                    omni_move.z = float(100.0) 

                    print("min_obs:", self.nav.min_dist_obs, self.nav.nav_target.adjust_min_dist)
                    
                    if self.nav.min_dist_obs < self.nav.nav_target.adjust_min_dist:
                        omni_move.x = float(0.0)
                        omni_move.y = float(0.0)
                        self.navigation_state = 2

                    self.omni_move_publisher.publish(omni_move)


                else:
                    # ERROR
                    self.navigation_state = 2


            if self.navigation_state == 2:
                # publica no topico a dizer que acabou
                print("Finished")
                finish_flag = Bool()
                finish_flag.data = True
                self.nav.first_nav_target = False
                omni_move = Vector3()
                omni_move.x = float(0.0)
                omni_move.y = float(0.0)
                omni_move.z = float(100.0)
                self.omni_move_publisher.publish(omni_move)
                # resets the acceleration value
                # if self.nav.nav_target.move_or_rotate.lower() == "adjust_angle":
                #     self.call_set_acceleration_ramp_server(1)
                self.flag_pos_reached_publisher.publish(finish_flag) 

            
            self.nav.update_debug_drawings() # this way it still draws the targets on the final frame


def main(args=None):
    rclpy.init(args=args)
    node = NavSDNLNode()
    rclpy.spin(node)
    rclpy.shutdown()
