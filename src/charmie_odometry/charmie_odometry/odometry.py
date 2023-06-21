#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from charmie_interfaces.msg import Encoders

import math

class RobotOdometry():
    def __init__(self):
        self.MAX_ENCODERS = 4294967295
        self.MOT_ENC = [0, 0, 0, 0]
        self.MOT_ENC_ANT = [0, 0, 0, 0]
        self.pulse_per_rotation = 980  # encoder pulses for a full wheel rotation
        self.wheel_diameter = 203  # mm
        self.robot_radius = 265  # this value is yet to be confimed by the 3D modulation of the robot !!!!!!!!!!
        # self.coord_rel_x = 0
        # self.coord_rel_y = 0
        self.coord_rel_t = 0
        self.coord_rel_x_ = 0 # coord_rel_x
        self.coord_rel_y_ = 0 # coord_rel_y

        # self.ctr = 0
        # self.flag_first_time = True
        # self.safety_rel_t = 0
        # self.safety_rel_x = 0
        # self.safety_rel_y = 0
    
    def localization(self, enc: Encoders):

        self.MOT_ENC[0] = enc.enc_m1
        self.MOT_ENC[1] = enc.enc_m2
        self.MOT_ENC[2] = enc.enc_m3
        self.MOT_ENC[3] = enc.enc_m4

        # for x,y in self.MOT_ENC, self.MOT_ENC_ANT:
        DIFF_MOT_ENC = [0, 0, 0, 0]

        # print(val_enc)

        for i in range(len(self.MOT_ENC)):
            DIFF_MOT_ENC[i] = self.MOT_ENC[i] - self.MOT_ENC_ANT[i]
            if DIFF_MOT_ENC[i] > self.MAX_ENCODERS/2:
                DIFF_MOT_ENC[i] -= self.MAX_ENCODERS + 1  # +1 because numerically the 0 to MAX_ENCODERS transition must be considered
            if DIFF_MOT_ENC[i] < -self.MAX_ENCODERS/2:
                DIFF_MOT_ENC[i] += self.MAX_ENCODERS + 1  # +1 because numerically the 0 to MAX_ENCODERS transition must be considered
            DIFF_MOT_ENC[i] *= -1

        # print("CURRENT:    ", self.MOT_ENC)
        # print("PREVIOUS:   ", self.MOT_ENC_ANT)
        # print("DIFFERENCE: ", DIFF_MOT_ENC)

        # acording to the Report of Inês Garcia and Tiago Ribeiro (P2 - 2017)
        # check report, named: C.H.A.R.M.I.E. - PLATAFORMA MÓVEL PARA ROBÔ DE SERVIÇOS EM CASA
        # the following equations calculate the odometry through encoders for a four wheeled omnidirectional platform

        # calculate the linear speed of each wheel

        d = [0, 0, 0, 0]
        v = [0, 0, 0, 0]
        for i in range(len(self.MOT_ENC)):
            d[i] = (((math.pi*self.wheel_diameter)/self.pulse_per_rotation)*self.MOT_ENC[i])
            v[i] = (((math.pi*self.wheel_diameter)/self.pulse_per_rotation)*DIFF_MOT_ENC[i])/0.05

        # print("v:", v)

        G_ = [0, 0]
        G = [0, 0]

        G_[0] = (v[1] + v[3]) / 2
        G_[1] = (v[0] + v[2]) / 2

        G[0] = ((math.sqrt(2) / 2) * (-G_[0] + G_[1]))  # + ((math.sqrt(2) / 2) * G_[1])
        G[1] = ((math.sqrt(2) / 2) * (G_[0] + G_[1]))  # + ((math.sqrt(2) / 2) * G_[1])

        # print("G_:", G_)
        # print("G:", G)

        vel_lin_enc = math.sqrt(G[0]**2 + G[1]**2)
        alfa_enc = math.pi/2 + math.pi/2 - math.atan2(G[1], -G[0])  # summed math.pi/2
        ### alfa_enc_deg = math.degrees(alfa_enc)
        # vel_ang_enc = (((G_[1] - v[0])+(G_[0] - v[3])) / 2) / self.robot_radius
        # vel_ang_enc = (((G_[1] - v[0]) + (G_[0] - v[1])) / 2) / self.robot_radius
        vel_ang_enc = (((G_[1] - v[0]) + (G_[0] - v[1]) + (-G_[1] + v[2]) + (-G_[0] + v[3])) / 4) / self.robot_radius

        # print(vel_ang_enc, vel_ang_enc2)

        # ##################### #
        #         90            #
        #     /---------\       #
        #     |         |       #
        # 180 |         | 360/0 #
        #     |         |       #
        #     \---------/       #
        #         270           #
        # ##################### #

        # print(round(alfa_enc_deg, 2))
        ### alfa_enc_deg = self.normalize_angles(alfa_enc_deg)

        # if alfa_enc_deg < 0:
        #     alfa_enc_deg += 360
        # print(vel_lin_enc, alfa_enc_deg, vel_ang_enc)
        # print(alfa_enc)

        ### xx = G[0]*0.05
        ### yy = G[1]*0.05
        ### theta = math.degrees(vel_ang_enc*0.05)
        theta = vel_ang_enc*0.05


        # print(xxx, yyy)

        ### self.coord_rel_x += xx
        ### self.coord_rel_y += yy
        self.coord_rel_t += theta

        ### self.coord_rel_t = self.normalize_angles(self.coord_rel_t)
        # tema que se normalizar o theta aqui!

        ### fi = alfa_enc_deg + self.coord_rel_t  # + self.coord_rel_t + (alfa_enc_deg)
        fi = alfa_enc + self.coord_rel_t ### + math.pi/2
        # print(math.degrees(fi), math.degrees(alfa_enc), math.degrees(self.coord_rel_t))

        ### fi = self.normalize_angles(fi)
        # while fi < 0:
        #     fi+=360
        # while fi >= 360:
        #     fi-=360

        # xx_ = xx * math.cos(math.radians(90 - self.coord_rel_t - alfa_enc_deg))
        # yy_ = yy * math.sin(math.radians(90 - self.coord_rel_t - alfa_enc_deg))
        ### xx_ = vel_lin_enc*0.05 * math.cos(math.radians(fi))
        ### yy_ = vel_lin_enc*0.05 * math.sin(math.radians(fi))
        xx_ = vel_lin_enc*0.05 * math.cos(fi)
        yy_ = vel_lin_enc*0.05 * math.sin(fi)

        self.coord_rel_x_ += xx_
        self.coord_rel_y_ += yy_

        # print(round(xx_, 2), round(yy_, 2), round(fi, 2), end='\t')

        """        
        print("XX:\t", round(self.coord_rel_x / 10, 2), "\tYY:\t", round(self.coord_rel_y / 10, 2), end='\t')
        print("X_:\t", round(self.coord_rel_x_ / 10, 2), "\tY_:\t", round(self.coord_rel_y_ / 10, 2),
              "\tTHETA(deg/rad):\t", round(self.normalize_angles(math.degrees(self.coord_rel_t)), 2),
               "\t", round(self.coord_rel_t, 2), end='\t|\t')

        print("ALFA(deg/rad):\t", round(self.normalize_angles(math.degrees(alfa_enc)), 2),  "\t", round(alfa_enc, 2),
              "\tVEL_LIN:\t", round(vel_lin_enc/10, 2), "\tVEL_ANG:\t", round(vel_ang_enc, 2))
        """

        self.MOT_ENC_ANT = self.MOT_ENC.copy()


        # end = time.time()
        # print(end - start, end=' ')
        # time_to_50ms = 0.05 - (end - start)
        # print(time_to_50ms)

        return self.coord_rel_x_, self.coord_rel_y_, self.coord_rel_t
        # return self.coord_rel_x_ + self.safety_rel_x, self.coord_rel_y_ + self.safety_rel_y, self.coord_rel_t + self.safety_rel_t

    def normalize_angles(self, ang):

        while ang < 0:
            ang += 360
        while ang >= 360:
            ang -= 360

        return ang


class OdometryNode(Node):

    def __init__(self):
        super().__init__("Odometry")
        self.get_logger().info("Initialised CHARMIE Odometry Node")

        self.encoders_subscriber = self.create_subscription(Encoders, "get_encoders", self.get_encoders_callback, 10)
        self.flag_encoders_publisher = self.create_publisher(Bool, "flag_encoders", 10)
        self.odometry_publisher = self.create_publisher(Odometry, "odom", 10)

        self.robot_odom = RobotOdometry()

        self.flag_enc = Bool()
        self.flag_enc.data = True
        self.flag_encoders_publisher.publish(self.flag_enc)

        
    def get_encoders_callback(self, enc: Encoders):
        self.coordinates = self.robot_odom.localization(enc) 
        print(self.coordinates)


    def normalize_angles(self, ang):

        while ang < 0:
            ang += 360
        while ang >= 360:
            ang -= 360

        return ang

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()
