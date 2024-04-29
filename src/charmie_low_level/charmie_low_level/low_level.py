#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Vector3
from example_interfaces.msg import Bool, Int16, Float32
from charmie_interfaces.msg import Encoders
import serial
import time
import struct


# TO DO:
#   change the way NumBytes work, these only make sense for variables that are requested
# to the MD49 boards, not for variables that are requested just to the low level board
#   when setting torso and legs position i need to send two variables, but the way it is
# made I can only send one command value, for MD49 I just needed to send one


# TO DO WHEN CHANGING TO SRV:
# - debug buttons



class RobotControl:

    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', baudrate=9600)  # open serial port
        print("Connected to Motor Board via:", self.ser.name)  # check which port was really used

        # FLAGS
        self.RESET_ENCODERS = {'EnableVar': 'r', 'DisableVar': 'r', 'Value': True}  # same value for enable and disable
        # self.REGULATOR = {'EnableVar': 'R', 'DisableVar': 'r', 'Value': True}
        self.TIMEOUT = {'EnableVar': 'T', 'DisableVar': 't', 'Value': True}
        self.MOVEMENT = {'EnableVar': 'G', 'DisableVar': 'g', 'Value': True}
        self.DEBUG_MOT_BOARD = {'EnableVar': 'K', 'DisableVar': 'k', 'Value': False}

        self.LIN_ACT_LEGS_ACTIVE = {'EnableVar': 'L', 'DisableVar': 'l', 'Value': False}
        self.LIN_ACT_TORSO_ACTIVE = {'EnableVar': 'C', 'DisableVar': 'c', 'Value': False}

        self.LIN_ACT_LEGS_MOVEM = {'EnableVar': '8', 'DisableVar': '2', 'Value': True}
        self.LIN_ACT_TORSO_MOVEM = {'EnableVar': '4', 'DisableVar': '6', 'Value': True}

        # NoByes = number of data bytes returned by just one motor board

        # GETS E SETS
        self.ACCELERATION = {'SetVar': 'A', 'GetVar': 'a', 'Value': [5, 5], 'NoBytes': 1, 'Min': 1, 'Max': 10}
        # self.MODE = {'SetVar': 'M', 'GetVar': 'm', 'Value': [0, 0], 'NoBytes': 1, 'Min': 0, 'Max': 0}  # has to be 0

        # GETS
        # self.VOLTS = {'GetVar': 'u', 'Value': [0, 0], 'NoBytes': 1}
        # self.VERSION = {'GetVar': 'v', 'Value': [0, 0], 'NoBytes': 1}
        self.ERROR = {'GetVar': 'x', 'Value': [0, 0], 'NoBytes': 1}
        # self.SPEEDS = {'GetVar': 's', 'Value': [0, 0, 0, 0], 'NoBytes': 2}
        # self.CURRENT = {'GetVar': 'i', 'Value': [0, 0, 0, 0], 'NoBytes': 2}
        # self.VI = {'GetVar': 'p', 'Value': [0, 0, 0, 0, 0, 0], 'NoBytes': 3}
        self.ENCODERS = {'GetVar': 'e', 'Value': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'NoBytes': 8}
        self.START_BUTTON = {'GetVar': 's', 'Value': [0, 0], 'NoBytes': 1}        
        self.DEBUG_BUTTONS = {'GetVar': 'b', 'Value': [0, 0], 'NoBytes': 1}
        self.VCCS = {'GetVar': 'u', 'Value': [0, 0], 'NoBytes': 1}
        self.LIN_ACT = {'GetVar': 'h', 'Value': [0, 0], 'NoBytes': 1}
        self.ORIENTATION = {'GetVar': 'o', 'Value': [0, 0], 'NoBytes': 1}
        self.IMU = {'GetVar': 'i', 'Value': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'NoBytes': 9}
        
        # SETS
        self.RGB = {'SetVar': 'Q', 'Value': [0], 'NoBytes': 1, 'Min': 0, 'Max': 255}
        self.LEGS = {'SetVar': 'S', 'Value': [0], 'NoBytes': 1, 'Min': 0, 'Max': 140}
        self.TORSO = {'SetVar': 'M', 'Value': [0], 'NoBytes': 1, 'Min': 8, 'Max': 170}
        
        self.LINEAR_V = {'SetVar': 'V', 'Value': [0], 'NoBytes': 1, 'Min': 0, 'Max': 100}
        self.ANGULAR_V = {'SetVar': 'W', 'Value': [100], 'NoBytes': 1, 'Min': 0, 'Max': 200}
        self.DIRECTION = {'SetVar': 'D', 'Value': [0], 'NoBytes': 1, 'Min': 0, 'Max': 359}  # NoBytes=2 since Max > 255

        # once the lienar actuators have the encoders we will have to change how they are controlled
        # self.LIN_ACT_LEGS_POS = {'SetVar': 'V', 'Value': [0], 'NoBytes': 1, 'Min': 0, 'Max': 100}
        # self.LIN_ACT_TORSO_POS = {'SetVar': 'V', 'Value': [0], 'NoBytes': 1, 'Min': 0, 'Max': 100}

        # OMNI MOVE
        self.OMNI_MOVE = {'CommVar': 'O', 'Lin': 0, 'Ang': 100, 'Dir': 0}
        self.OMNI_MOVE_ANT = {'Lin': 0, 'Ang': 100, 'Dir': 0}


    def set_omni_flags(self, comm_dict, num):
        comm_dict['Value'] = num
        if num is True:  # Enable
            # print(comm_dict['EnableVar'])
            self.ser.write(comm_dict['EnableVar'].encode('utf-8'))
        else:  # Disable
            # print(comm_dict['DisableVar'])
            self.ser.write(comm_dict['DisableVar'].encode('utf-8'))

    def set_omni_variables(self, comm_dict, num):
        if 'SetVar' in comm_dict:  # check if it is a variable that can be 'set'
            if comm_dict['Min'] <= num <= comm_dict['Max']:  # checks if the value received is within the correct range
                for i in range(len(comm_dict['Value'])):  # updates the value of the variable set, no need to get
                    comm_dict['Value'][i] = num
                # print(comm_dict['SetVar'], comm_dict['Value'])

                self.ser.write(comm_dict['SetVar'].encode('utf-8'))
                time.sleep(0.003)
                self.ser.write(comm_dict['Value'][0].to_bytes(comm_dict['NoBytes'], 'big'))
                return 0  # No error
            else:
                print("Invalid value! Out of bounds!")
                return -2  # Invalid value error
        else:
            print("Invalid SET Command! Please check if it makes sense!")
            return -1  # Invalid set command error

    def get_omni_variables(self, comm_dict):
        if 'GetVar' in comm_dict:  # check if it is a variable that can be 'get'
            # print(comm_dict['GetVar'], comm_dict['NoBytes'])
            self.ser.write(comm_dict['GetVar'].encode('utf-8'))  # sends get command
            time.sleep(0.003)

            while self.ser.in_waiting < comm_dict['NoBytes'] * 2:  # 2x NoBytes since there are two motor drivers
                pass  # waits until all the variables have been returned

            # print(self.ser.in_waiting)
            for i in range(comm_dict['NoBytes'] * 2):  # 2x NoBytes since there are two motor drivers
                x = ord(self.ser.read().decode('latin-1'))
                # x2 = ord(self.ser.read().decode('latin-1'))  # removed after fixing ChipKit sending \t after each var
                # print(x, end=', ')
                comm_dict['Value'][i] = x  # updates the value with the 'get' value
            return comm_dict['Value']  # returns the array of values

        else:
            print("Invalid GET Command! Please check if it makes sense!")
            return -1  # Invalid get command error

    def omni_move(self, dir_=None, lin_=None, ang_=None):

        if lin_ is None:  # checks if value is sent as function parameter, if not uses previous value
            self.OMNI_MOVE['Lin'] = self.LINEAR_V['Value'][0]
        elif self.LINEAR_V['Min'] <= lin_ <= self.LINEAR_V['Max']:  # checks if value is within the correct bounds
            self.OMNI_MOVE['Lin'] = lin_
            self.LINEAR_V['Value'][0] = lin_  # AFTER UPDATES !!!
        else:
            print("Invalid Linear Speed Value! Please check if it makes sense!")
            return -2  # returns out of bounds error

        if ang_ is None:  # checks if value is sent as function parameter, if not uses previous value
            self.OMNI_MOVE['Ang'] = self.ANGULAR_V['Value'][0]
        elif self.ANGULAR_V['Min'] <= ang_ <= self.ANGULAR_V['Max']:  # checks if value is within the correct bounds
            self.OMNI_MOVE['Ang'] = ang_
            self.ANGULAR_V['Value'][0] = ang_  # AFTER UPDATES !!!
        else:
            print("Invalid Angular Speed Value! Please check if it makes sense!")
            return -2  # returns out of bounds error

        if dir_ is None:  # checks if value is sent as function parameter, if not uses previous value
            self.OMNI_MOVE['Dir'] = self.DIRECTION['Value'][0]
        elif self.DIRECTION['Min'] <= dir_ <= self.DIRECTION['Max']:  # checks if value is within the correct bounds
            self.OMNI_MOVE['Dir'] = dir_
            self.DIRECTION['Value'][0] = dir_  # AFTER UPDATES !!!
        else:
            print("Invalid Direction Value! Please check if it makes sense!")
            return -2  # returns out of bounds error

        # FILTRO PARA NAO SEREM ENVIADOS TANTOS COMANDOS:" #######################################################
        # print("ANT1:", self.OMNI_MOVE_ANT)
        # print("CURR:", self.OMNI_MOVE)

        # dict_filter = lambda x, y: dict([(i, x[i]) for i in x if i in set(y)])
        # new_dict_keys = ('Dir,' 'Lin', 'Ang')
        # small_dict = dict_filter(self.OMNI_MOVE)

        # filter to adapt previous direction angle if near the 0/360
        thresh = 180  # 5  # or 180
        if self.OMNI_MOVE['Dir'] > 360 - thresh and self.OMNI_MOVE_ANT['Dir'] < 0 + thresh:
            self.OMNI_MOVE_ANT['Dir'] += 360
        elif self.OMNI_MOVE['Dir'] < 0 + thresh and self.OMNI_MOVE_ANT['Dir'] > 360 - thresh:
            self.OMNI_MOVE_ANT['Dir'] -= 360

        # print("ANT2:", self.OMNI_MOVE_ANT)

        # for key in ['Dir', 'Lin', 'Ang']:
        #     self.OMNI_MOVE_ANT[key] = self.OMNI_MOVE[key]






        # IMPORTANTE DESCOMENTAR
        """
        is_three_counter = 0
        for key in ['Dir', 'Lin', 'Ang']:
            if self.OMNI_MOVE_ANT[key] - 1 <= self.OMNI_MOVE[key] <= self.OMNI_MOVE_ANT[key] + 1:
                # print("OUT:", self.OMNI_MOVE[key], self.OMNI_MOVE_ANT[key], is_three_counter)
                is_three_counter += 1
            else:
                # print("IN:", self.OMNI_MOVE[key], self.OMNI_MOVE_ANT[key], is_three_counter)
                self.OMNI_MOVE_ANT[key] = self.OMNI_MOVE[key]
        if is_three_counter == 3:
            # print("OMNI MOVE NOT SENT")
            return -3  # returns similar motor value as before error
        """

        # print("SENT")

        """
        # bug quando a direcao varia entre 0 e 359
        if self.OMNI_MOVE_ANT['Dir'] - 1 <= self.OMNI_MOVE['Dir'] <= self.OMNI_MOVE_ANT['Dir'] + 1:
            return -3  # returns similar motor value as before error
        else:
            self.OMNI_MOVE_ANT['Dir'] = self.OMNI_MOVE['Dir']

        if self.OMNI_MOVE_ANT['Lin'] - 1 <= self.OMNI_MOVE['Lin'] <= self.OMNI_MOVE_ANT['Lin'] + 1:
            return -3  # returns similar motor value as before error
        else:
            self.OMNI_MOVE_ANT['Lin'] = self.OMNI_MOVE['Lin']

        if self.OMNI_MOVE_ANT['Ang'] - 1 <= self.OMNI_MOVE['Ang'] <= self.OMNI_MOVE_ANT['Ang'] + 1:
            return -3  # returns similar motor value as before error
        else:
            self.OMNI_MOVE_ANT['Ang'] = self.OMNI_MOVE['Ang']
        """

        # L2_ant = int(controller.L2dist * 100)
        # print(int(controller.L2dist * 100))

        dir_aux = int(255*self.OMNI_MOVE['Dir']/359)

        # print(type(self.OMNI_MOVE['Dir']))
        # print(type(dir_aux))
        
        #### MUST UNCOMMENT LATER, DEBUG ENCODERS
        # print('SENT VALUE')
        # print(self.OMNI_MOVE['CommVar'], " -> ", self.OMNI_MOVE['Dir'], self.OMNI_MOVE['Lin'], self.OMNI_MOVE['Ang'])
        
        
        # print(dir_aux)

        self.ser.write(self.OMNI_MOVE['CommVar'].encode('utf-8'))
        time.sleep(0.003)
        self.ser.write(dir_aux.to_bytes(1, 'big'))
        time.sleep(0.003)
        self.ser.write(self.OMNI_MOVE['Lin'].to_bytes(1, 'big'))
        time.sleep(0.003)
        self.ser.write(self.OMNI_MOVE['Ang'].to_bytes(1, 'big'))

        self.OMNI_MOVE_ANT['Lin'] = self.OMNI_MOVE['Lin']
        self.OMNI_MOVE_ANT['Ang'] = self.OMNI_MOVE['Ang']

class LowLevelNode(Node):

    def __init__(self):
        super().__init__("Low_Level")
        self.get_logger().info("Initialised CHARMIE Low Level Node")
        
        self.rgb_mode_subscriber = self.create_subscription(Int16, "rgb_mode", self.rgb_mode_callback , 10)
        
        self.start_button_publisher = self.create_publisher(Bool, "get_start_button", 10)
        self.flag_start_button_subscriber = self.create_subscription(Bool, "flag_start_button", self.flag_start_button_callback , 10)
        
        self.vccs_publisher = self.create_publisher(Pose2D, "get_vccs", 10)
        self.flag_vccs_subscriber = self.create_subscription(Bool, "flag_vccs", self.flag_vccs_callback , 10)

        self.torso_pos_subscriber = self.create_subscription(Pose2D, "torso_pos", self.torso_pos_callback , 10)
        self.get_torso_pos_publisher = self.create_publisher(Pose2D, "get_torso_pos", 10)
        self.flag_torso_pos_subscriber = self.create_subscription(Bool, "flag_torso_pos", self.flag_torso_pos_callback , 10)

        self.omni_move_subscriber = self.create_subscription(Vector3, "omni_move", self.omni_move_callback , 10)
        self.set_movement_subscriber = self.create_subscription(Bool, "set_movement", self.set_movement_callback , 10)

        self.get_encoders_publisher = self.create_publisher(Encoders, "get_encoders", 10)
        self.flag_encoders_subscriber = self.create_subscription(Bool, "flag_encoders", self.flag_encoders_callback , 10)

        self.get_orientation_publisher = self.create_publisher(Float32, "get_orientation", 10)
        self.flag_orientation_subscriber = self.create_subscription(Bool, "flag_orientation", self.flag_orientation_callback , 10)

        self.low_level_diagnostic_publisher = self.create_publisher(Bool, "low_level_diagnostic", 10)


        ### TEMP
        self.torso_test_subscriber = self.create_subscription(Pose2D, "torso_test", self.torso_test_callback , 10)




        flag_diagn = Bool()
        flag_diagn.data = False

        self.create_timer(0.1, self.timer_callback)
        # self.create_timer(1.0, self.timer_callback2)

        self.robot = RobotControl()

        self.robot.set_omni_flags(self.robot.RESET_ENCODERS, True)
        self.robot.set_omni_variables(self.robot.ACCELERATION, 1)
        self.robot.set_omni_flags(self.robot.TIMEOUT, False)
        self.robot.set_omni_variables(self.robot.RGB, 100)

        # test and reorganize
        aaa = self.robot.get_omni_variables(self.robot.ACCELERATION)

        print(aaa)
        # print(type(aaa))

        if aaa[0] == 100:
            self.get_logger().warning(f"Motors are not being powered!")
        else:
            self.get_logger().info(f"Connected to Motor Boards! Accel Ramp Lvl = {aaa[0]}")

        if aaa == [1, 1]:
            flag_diagn.data = True
        else:
            flag_diagn.data = False
        
        self.low_level_diagnostic_publisher.publish(flag_diagn)


        self.flag_get_start_button = False
        self.flag_get_vccs = False
        self.flag_get_torso_pos = False
        self.flag_get_encoders = False
        self.flag_get_orientation = False

    
    def torso_test_callback(self, data: Pose2D):

        print("receiving torso position ")
        estado_legs = int(data.x)
        estado_torso = int(data.y) 

        if estado_legs == 0:
            print("Legs Stopped")
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_ACTIVE, False)
            # nao anda
        elif estado_legs == 1:
            print("Legs Up")
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_ACTIVE, True)
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_MOVEM, True)
            # cima
        elif estado_legs == -1:
            print("Legs Down")
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_ACTIVE, True)
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_MOVEM, False)
            # baixo
        else:
            print("Legs Stopped 2")
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_ACTIVE, False)
            # nao amnda

        if estado_torso == 0:
            print("Torso Stopped")
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_ACTIVE, False)
            # nao anda
        elif estado_torso == 1:
            print("Torso Up")
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_ACTIVE, True)
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_MOVEM, True)
            # cima
        elif estado_torso == -1:
            print("Torso Down")
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_ACTIVE, True)
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_MOVEM, False)
            # baixo
        else:
            print("Torso Stopped 2")
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_ACTIVE, False)
            # nao amnda

    def timer_callback2(self):
            # request start button here
            # aux_s = self.robot.get_omni_variables(self.robot.START_BUTTON)
            # print("Start Button State: ", bool(aux_s[0]))

            # aux_v = self.robot.get_omni_variables(self.robot.VCCS)
            # print("VCC: ", aux_v[0]/5, " Emergency: ", bool(aux_v[1]))

            # aux_b = self.robot.get_omni_variables(self.robot.DEBUG_BUTTONS)
            # print("DEBUG BUTTONS STATE: ", aux_b[0], (aux_b[0] >> 0)&1, (aux_b[0] >> 1)&1, (aux_b[0] >> 2)&1)

            # aux_t = self.robot.get_omni_variables(self.robot.LIN_ACT)
            # print("LEGS HEIGHT:", aux_t[0], "TORSO ANGLE:", aux_t[1])

            aux_o = self.robot.get_omni_variables(self.robot.ORIENTATION)
            print("ORIENTATION:", (aux_o[0]<<8|aux_o[1])/10)

            aux_i = self.robot.get_omni_variables(self.robot.IMU)
            print("MAGX:", struct.unpack('!h', struct.pack('!BB', aux_i[0], aux_i[1]))[0],
                  "MAGY:", struct.unpack('!h', struct.pack('!BB', aux_i[2], aux_i[3]))[0],             
                  "MAGZ:", struct.unpack('!h', struct.pack('!BB', aux_i[4], aux_i[5]))[0],
                  "ACCX:", struct.unpack('!h', struct.pack('!BB', aux_i[6], aux_i[7]))[0],
                  "ACCY:", struct.unpack('!h', struct.pack('!BB', aux_i[8], aux_i[9]))[0],
                  "ACCZ:", struct.unpack('!h', struct.pack('!BB', aux_i[10], aux_i[11]))[0],
                  "GYRX:", struct.unpack('!h', struct.pack('!BB', aux_i[12], aux_i[13]))[0],
                  "GYRY:", struct.unpack('!h', struct.pack('!BB', aux_i[14], aux_i[15]))[0],
                  "GYRZ:", struct.unpack('!h', struct.pack('!BB', aux_i[16], aux_i[17]))[0],
                  )
            

    def timer_callback(self):

        if  self.flag_get_start_button:
            # request start button here
            aux_b = self.robot.get_omni_variables(self.robot.START_BUTTON)
            print("Start Button State: ", bool(aux_b[0]))

            cmd = Bool()
            cmd.data = bool(aux_b[0])
            self.start_button_publisher.publish(cmd)

        if  self.flag_get_vccs:
            # request vccs here
            aux_v = self.robot.get_omni_variables(self.robot.VCCS)
            print("VCC: ", aux_v[0]/10, " Emergency: ", bool(aux_v[1]))

            cmd = Pose2D()
            cmd.x = float(aux_v[0]/10)
            cmd.y = float(aux_v[1])
            self.vccs_publisher.publish(cmd)

        if  self.flag_get_torso_pos:
            # request torso pos here
            aux_t = self.robot.get_omni_variables(self.robot.LIN_ACT)
            print("Legs_pos: ", aux_t[0], " Torso_pos: ", aux_t[1])
        
            cmd = Pose2D()
            cmd.x = float(aux_t[0])
            cmd.y = float(aux_t[1])
            self.get_torso_pos_publisher.publish(cmd)

        if  self.flag_get_encoders:
            # request encoders here
            aux_e = self.robot.get_omni_variables(self.robot.ENCODERS)
            # print(aux_e)
            
            cmd = Encoders()
            cmd.enc_m1 = (aux_e[0] << 24) + (aux_e[1] << 16) + (aux_e[2] << 8) + aux_e[3]
            cmd.enc_m2 = (aux_e[4] << 24) + (aux_e[5] << 16) + (aux_e[6] << 8) + aux_e[7]
            cmd.enc_m3 = (aux_e[8] << 24) + (aux_e[9] << 16) + (aux_e[10] << 8) + aux_e[11]
            cmd.enc_m4 = (aux_e[12] << 24) + (aux_e[13] << 16) + (aux_e[14] << 8) + aux_e[15]

            print(aux_e[0], aux_e[1], aux_e[2], aux_e[3], "|", aux_e[4], aux_e[5], aux_e[6], aux_e[7], "|", aux_e[8], aux_e[9], aux_e[10], aux_e[11], "|", aux_e[12], aux_e[13], aux_e[14], aux_e[15])

            # print("Enc1: ", cmd.enc_m1, "Enc2: ", cmd.enc_m2, "Enc3: ", cmd.enc_m3, "Enc4: ", cmd.enc_m4)
            self.get_encoders_publisher.publish(cmd)

        if self.flag_get_orientation:

            aux_o = self.robot.get_omni_variables(self.robot.ORIENTATION)
            
            orientation = Float32()
            orientation.data = (aux_o[0]<<8|aux_o[1])/10
            
            print("ORIENTATION:", orientation.data)

            self.get_orientation_publisher.publish(orientation)


    def flag_start_button_callback(self, flag: Bool):
        # print("Flag Start Button Set To: ", flag.data)
        if flag.data:
            self.get_logger().info("Received Reading Start Button State True")
        else:
            self.get_logger().info("Received Reading Start Button State False")
        self.flag_get_start_button = flag.data

    def flag_vccs_callback(self, flag: Bool):
        # print("Flag VCCS Set To: ", flag.data)
        if flag.data:
            self.get_logger().info("Received Reading VCCs State True")
        else:
            self.get_logger().info("Received Reading VCCs State False")
        self.flag_get_vccs = flag.data

    def flag_torso_pos_callback(self, flag: Bool):
        # print("Flag Torso Set To: ", flag.data)
        if flag.data:
            self.get_logger().info("Received Reading Torso State True")
        else:
            self.get_logger().info("Received Reading Torso State False")
        self.flag_get_torso_pos = flag.data

    def flag_encoders_callback(self, flag: Bool):
        # print("Flag Encoders Set To: ", flag.data)
        if flag.data:
            self.get_logger().info("Received Reading Encoder State True")
        else:
            self.get_logger().info("Received Reading Encoder State False")
        self.flag_get_encoders = flag.data

    def flag_orientation_callback(self, flag: Bool):
        # print("Flag Orientation Set To: ", flag.data)
        if flag.data:
            self.get_logger().info("Received Reading Orientation State True")
        else:
            self.get_logger().info("Received Reading Orientation State False")
        self.flag_get_orientation = flag.data

    def set_movement_callback(self, flag: Bool):
        # print("Set Movement Set To: ", flag.data)
        if flag.data:
            self.get_logger().info("Received Set Movement State True")
        else:
            self.get_logger().info("Received Set Movement State False")
        self.robot.set_omni_flags(self.robot.MOVEMENT, flag.data)
        

    def rgb_mode_callback(self, mode: Int16):
        print("Received RGB mode: ", mode.data)
        self.robot.set_omni_variables(self.robot.RGB, mode.data)

    def torso_pos_callback(self, pos: Pose2D):
        print("Received TORSO pos:", pos.x, pos.y)
        self.robot.set_omni_variables(self.robot.LEGS, int(pos.x))
        self.robot.set_omni_variables(self.robot.TORSO, int(pos.y))

    def omni_move_callback(self, omni:Vector3):
        ### MUST UNCOMMENT LATER, TESTING DEBUG 
        # print("Received OMNI move. dir =", omni.x, "vlin =", omni.y, "vang =", omni.z)
        self.robot.omni_move(dir_= int(omni.x), lin_= int(omni.y), ang_= int(omni.z))


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelNode()
    rclpy.spin(node)
    rclpy.shutdown()
