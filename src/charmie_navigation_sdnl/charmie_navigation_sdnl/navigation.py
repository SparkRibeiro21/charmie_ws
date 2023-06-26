#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from charmie_interfaces.msg import TarNavSDNL, Obstacles

import cv2
import numpy as np
import math

class NavigationSDNLClass:

    def __init__(self):

        self.lambda_target = 10
        self.beta1 = 30
        self.beta2 = 40

        self.obstacles = Obstacles()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0
        self.nav_target = TarNavSDNL()
        self.first_nav_target = False

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
        self.v_max = 30


        # visual debug
        self.DEBUG_DRAW_IMAGE = True # debug drawing opencv
        self.xc = 400
        self.yc = 400
        self.test_image = np.zeros((self.xc*2, self.yc*2, 3), dtype=np.uint8)
        self.image_plt = np.zeros((self.xc * 2, self.yc * 2, 3), dtype=np.uint8)
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

    def sdnl_main(self):

        self.f_target, self.y_atrator = self.atrator()
        

        #### PROXIMAS DUAS FUNCOES DEVIAM ESTAR DENTRO DO IF POR UMA QUESTAO DE EFICIENCIA
        #### POREM POR DEBUG PODEM ESTAR CA FORA
        self.obstacles_l = self.calculate_obstacles_lidar()

        # self.f_obstacle, self.y_repulsor1, self.y_repulsor2, self.y_repulsor3, self.yf, self.yff, self.yfff = self.repulsor()
        self.f_obstacle, self.yff, self.yfff = self.repulsor()
        
        if not self.nav_target.flag_not_obs:
            self.f_final, self.y_final = self.combine_atrator_repulsor()
            print("ATRATOR + REPULSORES")
        else:
            # in case it is intended to not consider obstacles
            self.f_final = self.f_target
            self.y_final = self.y_atrator
            print("SÓ ATRATOR")

        print(self.f_final)

        if self.f_final > self.v_max:
            self.f_final = self.v_max
        if self.f_final < -self.v_max:
            self.f_final = -self.v_max



        # print("Received OMNI move. dir =", omni.x, "vlin =", omni.y, "vang =", omni.z)
        omni_move = Vector3()
        omni_move.y = 10
        omni_move.z = 100 - int(self.f_final)
        
        return omni_move
        


    def atrator(self):

        # variables necessary to calculate the atrator
        target = (self.nav_target.move_target_coordinates.x, self.nav_target.move_target_coordinates.y)
        current_pos = (self.robot_x, self.robot_y)
        phi = self.robot_t


        phi_ = phi + math.pi/2


        psi_target = math.atan2(target[1]-current_pos[1], target[0]-current_pos[0])

        f_target = -self.lambda_target*math.sin(phi_ - psi_target)

        # print(math.degrees(psi_target), phi)
        # print(f_target)

        x = np.linspace(0, 2 * math.pi, 360 * 2)
        y = [0] * 360*2

        for half_degree in range(360*2):
            y[half_degree] = -self.lambda_target*math.sin(x[half_degree] - psi_target)
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
        phi_ = phi + math.pi/2

        for obs in self.obstacles.obstacles:
            # print(obs)
            aux_obs['psi_obs'] = phi_ + obs.alfa
            aux_obs['dis_obs'] = obs.dist
            aux_obs['del_tet'] = obs.length_degrees

            obstacles.append(aux_obs.copy())

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

            f_obs = lambda_obstacle_ * (phi_ - psi_obstacle) * math.exp(-((phi_ - psi_obstacle) ** 2) / (2 * sigma_ ** 2))
            f_obstacle += f_obs
            # print(f_obs)


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


    def update_debug_drawings(self):
            
        if self.DEBUG_DRAW_IMAGE:

            # 1 meter lines horizontal and vertical
            for i in range(10):
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
            self.all_obs_val.append(self.obstacles)
            
            # past obstacles
            for j in range(len(self.all_obs_val)):

                for i in range(self.all_obs_val[j].no_obstacles):

                    # aux variables
                    aux_ang = self.all_obs_val[j].obstacles[i].alfa
                    aux_dist = self.all_obs_val[j].obstacles[i].dist + self.robot_radius
                    aux_len_cm = self.all_obs_val[j].obstacles[i].length_cm
                    thickness = 20

                    # line length of obstacle at dist and alfa detected
                    cv2.line(self.test_image, (int(self.xc + self.scale*self.all_pos_x_val[j] - self.scale * (aux_dist) * (math.cos(aux_ang - self.all_pos_t_val[j] + math.pi/2)) + self.scale * (aux_len_cm/2) * math.cos(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2))),
                                               int(self.yc - self.scale*self.all_pos_y_val[j] - self.scale * (aux_dist) * (math.sin(aux_ang - self.all_pos_t_val[j] + math.pi/2)) - self.scale * (aux_len_cm/2) * math.sin(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2)))),
                                              (int(self.xc + self.scale*self.all_pos_x_val[j] - self.scale * (aux_dist) * (math.cos(aux_ang - self.all_pos_t_val[j] + math.pi/2)) - self.scale * (aux_len_cm/2) * math.cos(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2))),
                                               int(self.yc - self.scale*self.all_pos_y_val[j] - self.scale * (aux_dist) * (math.sin(aux_ang - self.all_pos_t_val[j] + math.pi/2)) + self.scale * (aux_len_cm/2) * math.sin(-(math.pi/2 + aux_ang - self.all_pos_t_val[j] + math.pi/2)))),
                                              (0, 165, 255), int(1.0 + thickness*self.scale/1000))
             

            thickness = 20
            # current obstacles
            for i in range(self.obstacles.no_obstacles):

                # aux variables
                aux_ang = self.obstacles.obstacles[i].alfa
                aux_dist = self.obstacles.obstacles[i].dist + self.robot_radius
                aux_len_cm = self.obstacles.obstacles[i].length_cm

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
                cv2.line(self.test_image, (int(self.xc + self.scale*self.robot_x - self.scale * (aux_dist) * (math.cos(aux_ang - self.robot_t + math.pi/2)) + self.scale * (aux_len_cm/2) * math.cos(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2))),
                                        int(self.yc - self.scale*self.robot_y - self.scale * (aux_dist) * (math.sin(aux_ang - self.robot_t + math.pi/2)) - self.scale * (aux_len_cm/2) * math.sin(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2)))),
                                        (int(self.xc + self.scale*self.robot_x - self.scale * (aux_dist) * (math.cos(aux_ang - self.robot_t + math.pi/2)) - self.scale * (aux_len_cm/2) * math.cos(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2))),
                                        int(self.yc - self.scale*self.robot_y - self.scale * (aux_dist) * (math.sin(aux_ang - self.robot_t + math.pi/2)) + self.scale * (aux_len_cm/2) * math.sin(-(math.pi/2 + aux_ang - self.robot_t + math.pi/2)))),
                                        (0, 0, 255), int(1.0 + thickness*self.scale/1000))
                  

            
            # targets
            if self.first_nav_target:
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
                            
            
            # robot
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x), int(self.yc - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x), int(self.yc - self.scale * self.robot_y)), (int)(self.scale*self.robot_radius/10), (0, 255, 255), 1)
            cv2.circle(self.test_image, (int(self.xc + self.scale*self.robot_x + (self.robot_radius - self.lidar_radius)*self.scale*math.cos(self.robot_t + math.pi/2)),
                                            int(self.yc - self.scale*self.robot_y - (self.robot_radius - self.lidar_radius)*self.scale*math.sin(self.robot_t + math.pi/2))), (int)(self.scale*self.lidar_radius), (0, 255, 255), -1)
            


            # SDNL equations
            if self.first_nav_target:
                self.plot1.plot(self.image_plt, self.scale_plotter, self.y_atrator, self.robot_t)
            
            # self.plot2.plot(self.image_plt, self.scale_plotter, self.y_repulsor1, self.robot_t, (255, 200, 200))
            # self.plot2.plot(self.image_plt, self.scale_plotter, self.y_repulsor2, self.robot_t, (255, 100, 100))
            # self.plot2.plot(self.image_plt, self.scale_plotter, self.y_repulsor3, self.robot_t, (255, 0, 0))

            # self.plot3.plot(self.image_plt, self.scale_plotter, self.y_repulsor2, self.robot_t, (255, 100, 100))


            for y_plt_ff in self.yff:
                self.plot3.plot(self.image_plt, self.scale_plotter, y_plt_ff, self.robot_t, (255, 100, 100))
            self.plot2.plot(self.image_plt, self.scale_plotter, self.yfff, self.robot_t, (0, 140, 255))
            self.plot2.plot(self.image_plt, self.scale_plotter, self.y_final, self.robot_t, (255, 255, 0))
        

                # if FLAG_MODULE_LIDAR:
                #    for y_plt_ff in yff:
                #         plot3.plot(self.image_plt, scale_plotter, y_plt_ff, coord_rel_t, (255, 100, 100))
                #     plot2.plot(self.image_plt, scale_plotter, yfff, coord_rel_t, (0, 140, 255))
                #     plot2.plot(self.image_plt, scale_plotter, y_final, coord_rel_t, (255, 255, 0))



            cv2.imshow("Navigation SDNL", self.test_image)
            cv2.imshow("SDNL", self.image_plt)
            
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


    def odometry_msg_to_position(self, odom: Odometry):
        
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y

        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w

        # yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        # pitch = math.asin(-2.0*(qx*qz - qw*qy))
        # roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        # print(yaw, pitch, roll)

        self.robot_t = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        # print(self.robot_x, self.robot_y, self.robot_t)

    def obstacles_msg_to_position(self, obs: Obstacles):
        self.obstacles = obs
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

        for value in range(len(self.x)):
            # print(value, end="")
            # cv2.circle(image_plt, (image_plt.shape[1] - self.centre_data - value, int(self.coord_y - scale/10 * y[value])), 0, (100,100,100), 0)
            cv2.circle(image_plt, (self.centre_data + value, int(self.coord_y - scale/20 * y[value])), 0, fcolour, 0)

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

        # Create PUBs/SUBs
        self.obs_lidar_subscriber = self.create_subscription(Obstacles, "obs_lidar", self.obs_lidar_callback, 10)
        self.odom_robot_subscriber = self.create_subscription(Odometry, "odom_robot", self.odom_robot_callback, 10)
        self.omni_move_publisher = self.create_publisher(Vector3, "omni_move", 10)
        
        self.target_pos_subscriber = self.create_subscription(TarNavSDNL, "target_pos", self.target_pos_callback, 10)
        self.flag_pos_reached_publisher = self.create_publisher(Bool, "flag_start_button", 10)


    def obs_lidar_callback(self, obs: Obstacles):
        # updates the obstacles variable
        self.nav.obstacles_msg_to_position(obs)
        self.nav.sdnl_main()
        self.nav.update_debug_drawings()

    def odom_robot_callback(self, odom: Odometry):
        # updates the position variable
        self.nav.odometry_msg_to_position(odom)
        self.nav.sdnl_main()
        self.nav.update_debug_drawings()

    def target_pos_callback(self, nav: TarNavSDNL):
        # calculates the velocities and sends them to the motors considering the latest obstacles and odometry position
        self.nav.navigation_msg_to_position(nav)
        omni_move = self.nav.sdnl_main()
        self.omni_move_publisher.publish(omni_move)
        self.nav.update_debug_drawings()
        print(nav)


def main(args=None):
    rclpy.init(args=args)
    node = NavSDNLNode()
    rclpy.spin(node)
    rclpy.shutdown()
