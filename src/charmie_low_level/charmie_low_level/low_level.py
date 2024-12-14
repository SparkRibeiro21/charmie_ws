#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Vector3
from example_interfaces.msg import Bool, Int16, Float32
from charmie_interfaces.msg import Encoders
from charmie_interfaces.srv import SetAcceleration, SetRGB, GetLowLevelButtons, GetVCCs, GetTorso, SetTorso, ActivateBool
import serial
import time
import struct

# TO DO:
#   change the way NumBytes work, these only make sense for variables that are requested
# to the MD49 boards, not for variables that are requested just to the low level board
#   when setting torso and legs position i need to send two variables, but the way it is
# made I can only send one command value, for MD49 I just needed to send one

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
        # self.START_BUTTON = {'GetVar': 's', 'Value': [0, 0], 'NoBytes': 1} # NOT USED BUT OPERATIONAL
        # self.DEBUG_BUTTONS = {'GetVar': 'd', 'Value': [0, 0], 'NoBytes': 1} # NOT USED BUT OPERATIONAL
        self.ALL_BUTTONS = {'GetVar': 'b', 'Value': [0, 0], 'NoBytes': 1}
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
        
        # Torso
        self.torso_move_subscriber = self.create_subscription(Pose2D, "torso_move", self.torso_move_callback , 10)
        # Motors
        self.omni_move_subscriber = self.create_subscription(Vector3, "omni_move", self.omni_move_callback , 10)
        # Encoders
        self.get_encoders_publisher = self.create_publisher(Encoders, "get_encoders", 10)
        # IMU
        self.get_orientation_publisher = self.create_publisher(Float32, "get_orientation", 10)
        
        ### Services (Clients) ###
        # Acceleration
        self.server_set_acceleration = self.create_service(SetAcceleration, "set_acceleration_ramp", self.callback_set_acceleration) 
        # RGB 
        self.server_set_rgb = self.create_service(SetRGB, "rgb_mode", self.callback_set_rgb) 
        # VCCs
        self.server_vccs = self.create_service(GetVCCs, "get_vccs", self.callback_get_vccs)
        # Start Button and Debug Buttons
        self.server_start_button = self.create_service(GetLowLevelButtons, "get_start_button", self.callback_get_start_button)
        # Torso
        self.server_get_torso_position = self.create_service(GetTorso, "get_torso_position", self.callback_get_torso_position)
        self.server_set_torso_position = self.create_service(SetTorso, "set_torso_position", self.callback_set_torso_position)
        # IMU
        self.activate_orientation = self.create_service(ActivateBool, "activate_orientation", self.callback_activate_orientation)
        # Encoders
        self.activate_encoders = self.create_service(ActivateBool, "activate_encoders", self.callback_activate_encoders)
        # Motors
        self.activate_motors = self.create_service(ActivateBool, "activate_motors", self.callback_activate_motors)


        self.create_timer(0.1, self.timer_callback)
        # self.create_timer(1.0, self.timer_callback2)

        self.robot = RobotControl()

        self.robot.set_omni_flags(self.robot.RESET_ENCODERS, True)
        self.robot.set_omni_variables(self.robot.ACCELERATION, 1)
        self.robot.set_omni_flags(self.robot.TIMEOUT, False)
        self.robot.set_omni_variables(self.robot.RGB, 100)

        # test and reorganize
        aaa = self.robot.get_omni_variables(self.robot.ACCELERATION)
        # print(aaa)
        
        if aaa[0] == 100:
            self.get_logger().warning(f"Motors are not being powered!")
        else:
            self.get_logger().info(f"Connected to Motor Boards! Accel Ramp Lvl = {aaa[0]}")

        self.flag_get_encoders = False
        self.flag_get_orientation = False

    
    def callback_set_acceleration(self, request, response):
        # print(request)

        # Type of service received:
        # bool activate_lidar_up     # activate lidar from robot body
        # bool activate_lidar_bottom # activate lidar to see floor objects
        # bool activate_camera_head  # activate head camera for 3D obstacles  
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.
        self.get_logger().info("Received Set Acceleration Ramp %s" %("("+str(request.data)+")"))
        self.robot.set_omni_variables(self.robot.ACCELERATION, request.data)
        print(self.robot.get_omni_variables(self.robot.ACCELERATION))

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Set Acceleration Ramp for Motors to " + str(request.data)
        return response

    def callback_set_rgb(self, request, response):
        # print(request)

        # Type of service received:
        # int32 colour   # value of colour using the constant variables to ease RGB_MODE encoding 
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        if request.colour >= 0:
            
            colour_str = self.colour_to_string(request.colour)
            self.get_logger().info("Received Set RGB: %s" %(colour_str+"("+str(request.colour)+")"))
            self.robot.set_omni_variables(self.robot.RGB, request.colour)

            # returns whether the message was played and some informations regarding status
            response.success = True
            response.message = "Set RGB to " + colour_str

        else:

            # returns whether the message was played and some informations regarding status
            response.success = False
            response.message = "Error - Invalid RGB Value Received."

        return response

    def callback_get_vccs(self, request, response):
        # print(request)

        # Type of service received:
        # ---
        # float64 battery_voltage # battery voltage level 
        # bool emergency_stop     # boolean info of the emergency stop button

        self.get_logger().info("Received Get VCCs")
        aux_v = self.robot.get_omni_variables(self.robot.VCCS)

        # returns the values requested by the get_vccs
        response.battery_voltage = ((aux_v[0]/10)*2)+1.0
        response.emergency_stop = bool(aux_v[1])

        return response

    def callback_get_start_button(self, request, response):
        # print(request)

        # Type of service received:
        # ---
        # bool start_button  # start button state 
        # bool debug_button1 # debug1 button state boolean
        # bool debug_button2 # debug2 button state boolean
        # bool debug_button3 # debug3 button state boolean

        self.get_logger().info("Received Get Start and Debug Buttons")
        aux_b = self.robot.get_omni_variables(self.robot.ALL_BUTTONS)
        # print(aux_b[0])

        # returns the values requested by the get_low_level_buttons
        response.start_button  = bool((aux_b[0] >> 3) & 1)
        response.debug_button1 = bool((aux_b[0] >> 0) & 1)
        response.debug_button2 = bool((aux_b[0] >> 1) & 1)
        response.debug_button3 = bool((aux_b[0] >> 2) & 1)
        # print(response.start_button, response.debug_button1, response.debug_button2, response.debug_button3)

        return response

    def callback_get_torso_position(self, request, response):
        # print(request)

        # Type of service received:
        # ---
        # int32 legs  # up and down torso movement 
        # int32 torso # take a bow torso movement 
        
        self.get_logger().info("Received Get Torso Position")
        aux_t = self.robot.get_omni_variables(self.robot.LIN_ACT)
        print("Legs_pos: ", aux_t[0], " Torso_pos: ", aux_t[1])

        # returns the values requested by the get_torso_position
        response.legs = aux_t[0] # up and down torso movement 
        response.torso = aux_t[1] # take a bow torso movement 
        # print(response.legs, response.torso)

        return response
    
    def callback_set_torso_position(self, request, response):
        # print(request)

        # Type of service received:
        # int32 legs  # up and down torso movement 
        # int32 torso # take a bow torso movement 
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        self.get_logger().info("Received Set Torso Position")
        self.robot.set_omni_variables(self.robot.LEGS, request.legs)
        self.robot.set_omni_variables(self.robot.TORSO, request.torso)
        print("Legs_pos: ", request.legs, " Torso_pos: ", request.torso)

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Set torso L: " + str(request.legs) + ", T: " + str(request.torso)
        return response
    
    def callback_activate_orientation(self, request, response):
        # print(request)

        # Type of service received:
        # bool activate   # activate or deactivate
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.

        self.get_logger().info("Received Activate Orientation: %s" %(request.activate))
        self.flag_get_orientation = request.activate

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Sucessfully Activated Orientation to: " + str(request.activate)
        return response
    
    def callback_activate_encoders(self, request, response):
        # print(request)

        # Type of service received:
        # bool activate   # activate or deactivate
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.

        self.get_logger().info("Received Activate Encoders: %s" %(request.activate))
        self.flag_get_encoders = request.activate

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Sucessfully Activated Encoders to: " + str(request.activate)
        return response
    
    def callback_activate_motors(self, request, response):
        # print(request)

        # Type of service received:
        # bool activate   # activate or deactivate
        # ---
        # bool success    # indicate successful run of triggered service
        # string message  # informational, e.g. for error messages.

        self.get_logger().info("Received Activate Motors: %s" %(request.activate))
        self.robot.set_omni_flags(self.robot.MOVEMENT, request.activate)

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Sucessfully Activated Motors to: " + str(request.activate)
        return response

    def omni_move_callback(self, omni:Vector3):
        ### MUST UNCOMMENT LATER, TESTING DEBUG 
        # print("Received OMNI move. dir =", omni.x, "vlin =", omni.y, "vang =", omni.z)
        self.robot.omni_move(dir_= int(omni.x), lin_= int(omni.y), ang_= int(omni.z))


    def timer_callback(self):

        if  self.flag_get_encoders:
            aux_e = self.robot.get_omni_variables(self.robot.ENCODERS)
            # print(aux_e)
            cmd = Encoders()
            cmd.enc_m1 = (aux_e[0] << 24) + (aux_e[1] << 16) + (aux_e[2] << 8) + aux_e[3]
            cmd.enc_m2 = (aux_e[4] << 24) + (aux_e[5] << 16) + (aux_e[6] << 8) + aux_e[7]
            cmd.enc_m3 = (aux_e[8] << 24) + (aux_e[9] << 16) + (aux_e[10] << 8) + aux_e[11]
            cmd.enc_m4 = (aux_e[12] << 24) + (aux_e[13] << 16) + (aux_e[14] << 8) + aux_e[15]
            # print(aux_e[0], aux_e[1], aux_e[2], aux_e[3], "|", aux_e[4], aux_e[5], aux_e[6], aux_e[7], "|", aux_e[8], aux_e[9], aux_e[10], aux_e[11], "|", aux_e[12], aux_e[13], aux_e[14], aux_e[15])
            # print("Enc1: ", cmd.enc_m1, "Enc2: ", cmd.enc_m2, "Enc3: ", cmd.enc_m3, "Enc4: ", cmd.enc_m4)
            self.get_encoders_publisher.publish(cmd)

        if self.flag_get_orientation:
            aux_o = self.robot.get_omni_variables(self.robot.ORIENTATION)
            orientation = Float32()
            orientation.data = (aux_o[0]<<8|aux_o[1])/10
            # print("ORIENTATION:", orientation.data)
            self.get_orientation_publisher.publish(orientation)


    def torso_move_callback(self, data: Pose2D): # used by ps4 controller

        # print("receiving torso position ")
        estado_legs = int(data.x)
        estado_torso = int(data.y) 

        if estado_legs == 0:
            # print("Legs Stopped")
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_ACTIVE, False)
            # nao anda
        elif estado_legs == 1:
            # print("Legs Up")
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_ACTIVE, True)
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_MOVEM, True)
            # cima
        elif estado_legs == -1:
            # print("Legs Down")
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_ACTIVE, True)
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_MOVEM, False)
            # baixo
        else:
            # print("Legs Stopped 2")
            self.robot.set_omni_flags(self.robot.LIN_ACT_LEGS_ACTIVE, False)
            # nao amnda

        if estado_torso == 0:
            # print("Torso Stopped")
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_ACTIVE, False)
            # nao anda
        elif estado_torso == 1:
            # print("Torso Up")
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_ACTIVE, True)
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_MOVEM, True)
            # cima
        elif estado_torso == -1:
            # print("Torso Down")
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_ACTIVE, True)
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_MOVEM, False)
            # baixo
        else:
            # print("Torso Stopped 2")
            self.robot.set_omni_flags(self.robot.LIN_ACT_TORSO_ACTIVE, False)
            # nao amnda

    # here for when we add topic for readings all 9 axis of localizatio n IMU
    """
    def timer_callback2(self):

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
    """   

    def colour_to_string(self, colour=0):
        
        # Constant Variables to ease RGB_MODE coding
        # RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE, ORANGE, PINK, BROWN  = 0, 10, 20, 30, 40, 50, 60, 70, 80, 90
        # SET_COLOUR, BLINK_LONG, BLINK_QUICK, ROTATE, BREATH, ALTERNATE_QUARTERS, HALF_ROTATE, MOON, BACK_AND_FORTH_4, BACK_AND_FORTH_8  = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
        # CLEAR, RAINBOW_ROT, RAINBOW_ALL, POLICE, MOON_2_COLOUR, PORTUGAL_FLAG, FRANCE_FLAG, NETHERLANDS_FLAG = 255, 100, 101, 102, 103, 104, 105, 106

        final_str = ""

        if colour < 100:
            
            temp_clr = (int)(colour/10)
            temp_mode = (int)(colour%10)
            # print(temp_clr, temp_mode)

            match temp_clr:
                case 0:
                    colour_str = "Red"
                case 1:
                    colour_str = "Green"
                case 2:
                    colour_str = "Blue"
                case 3:
                    colour_str = "Yellow"
                case 4:
                    colour_str = "Magenta"
                case 5:
                    colour_str = "Cyan"
                case 6:
                    colour_str = "White"
                case 7:
                    colour_str = "Orange"
                case 8:
                    colour_str = "Pink"
                case 9:
                    colour_str = "Brown"

            match temp_mode:
                case 0:
                    mode_str = "Set_Colour"
                case 1:
                    mode_str = "Blink_Long"
                case 2:
                    mode_str = "Blink_Quick"
                case 3:
                    mode_str = "Rotate"
                case 4:
                    mode_str = "Breath"
                case 5:
                    mode_str = "Alternate_Quarters"
                case 6:
                    mode_str = "Half_Rotate"
                case 7:
                    mode_str = "Moon"
                case 8:
                    mode_str = "Back_and_Forth_4"
                case 9:
                    mode_str = "Back_and_Forth_8"

            final_str = colour_str + "+" + mode_str

        else: # custom modes

            match colour:
                case 255:
                    colour_str = "Clear"
                case 100:
                    colour_str = "Rainbow_Rotate"
                case 101:
                    colour_str = "Rainbow_All"
                case 102:
                    colour_str = "Police"
                case 103:
                    colour_str = "Moon_to_Colour"
                case 104:
                    colour_str = "Portugal_Flag"
                case 105:
                    colour_str = "France_Flag"
                case 106:
                    colour_str = "Netherlands_Flag"
                case _: # default
                    colour_str = "ERRROR:COLOUR_NOT_EXIST"

            final_str = colour_str

        return final_str
    

def main(args=None):
    rclpy.init(args=args)
    node = LowLevelNode()
    rclpy.spin(node)
    rclpy.shutdown()
