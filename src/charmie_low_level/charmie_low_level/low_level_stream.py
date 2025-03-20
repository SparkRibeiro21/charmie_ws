#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Vector3, Twist
from example_interfaces.msg import Bool, Int16, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from charmie_interfaces.msg import Encoders, ErrorsMotorBoard, ButtonsLowLevel, VCCsLowLevel, TorsoPosition
from charmie_interfaces.srv import SetAcceleration, SetRGB, GetLowLevelButtons, GetVCCs, GetTorso, SetTorso, ActivateBool
import serial
import time
import struct
import math

# TO DO:
#   change the way NumBytes work, these only make sense for variables that are requested
# to the MD49 boards, not for variables that are requested just to the low level board
#   when setting torso and legs position i need to send two variables, but the way it is
# made I can only send one command value, for MD49 I just needed to send one

class RobotControl:

    def __init__(self):
        
        # open serial port by id, this removes problems with volative port names: "/dev/ttyUSBX"
        self.ser = serial.Serial('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', baudrate=9600)
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

        self.COMMS_DELAY = 0.003

        self.start_time = time.time()
        self.data_stream = []
        self.first_data_stream = False


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
                time.sleep(self.COMMS_DELAY)
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
            time.sleep(self.COMMS_DELAY)

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
        time.sleep(self.COMMS_DELAY)
        self.ser.write(dir_aux.to_bytes(1, 'big'))
        time.sleep(self.COMMS_DELAY)
        self.ser.write(self.OMNI_MOVE['Lin'].to_bytes(1, 'big'))
        time.sleep(self.COMMS_DELAY)
        self.ser.write(self.OMNI_MOVE['Ang'].to_bytes(1, 'big'))

        self.OMNI_MOVE_ANT['Lin'] = self.OMNI_MOVE['Lin']
        self.OMNI_MOVE_ANT['Ang'] = self.OMNI_MOVE['Ang']

        
    def check_data_stream(self):

        number_of_bytes_in_a_batch_of_data = 29

        # checks whether has received the number of bytes equivalente to one batch of data
        if self.ser.in_waiting >= number_of_bytes_in_a_batch_of_data:

            # print("time:", time.time() - self.start_time)
            # print("@@@")
            # print(self.ser.in_waiting)

            ### check for start of comms protocol bytes
            correct_commms_protocol = False
            while not correct_commms_protocol:
                if self.ser.in_waiting >= number_of_bytes_in_a_batch_of_data:
                    x1 = ord(self.ser.read().decode('latin-1'))
                    if x1 == 35:
                        x2 = ord(self.ser.read().decode('latin-1'))
                        if x2 == 35:
                            correct_commms_protocol = True

            
            if not self.first_data_stream:
                self.first_data_stream = True
                        
            # print("@@@")
            # print(self.ser.in_waiting)

            self.data_stream.clear()

            ctr = 0
            for i in range(number_of_bytes_in_a_batch_of_data-2):
                x = ord(self.ser.read().decode('latin-1'))
                self.data_stream.append(x)
                ctr+=1

            # print("ctr = ", ctr)
            print(self.data_stream)
            print(self.ser.in_waiting)

            # Clear the serial buffer after reading
            # self.ser.reset_input_buffer()

            # print(self.ser.in_waiting)
            # self.start_time = time.time()
        else:
            print("WAITING")


class LowLevelNode(Node):

    def __init__(self):
        super().__init__("Low_Level")
        self.get_logger().info("Initialised CHARMIE Low Level Node")
        
        ### TOPICS ### 
        # Torso
        self.torso_move_subscriber = self.create_subscription(Pose2D, "torso_move", self.torso_move_callback , 10)
        # Motors
        self.omni_move_subscriber = self.create_subscription(Vector3, "omni_move", self.omni_move_callback , 10)
        self.cmd_vel_subscriber = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback , 10)
        # Errors
        self.errors_low_level_publisher = self.create_publisher(ErrorsMotorBoard, "errors_low_level", 10)
        # Encoders
        self.wheel_encoders_low_level_publisher = self.create_publisher(Odometry, "wheel_encoders", 10)
        # Buttons
        self.buttons_low_level_publisher = self.create_publisher(ButtonsLowLevel, "buttons_low_level", 10)
        # VCCs
        self.vccs_low_level_publisher = self.create_publisher(VCCsLowLevel, "vccs_low_level", 10)
        # Torso
        self.torso_low_level_publisher = self.create_publisher(TorsoPosition, "torso_position", 10)
        # Orientation (IMU)
        self.orientation_low_level_publisher = self.create_publisher(Float32, "orientation_low_level", 10)
        # IMU (for planar robot, just the basic for efficiency)
        self.imu_base_low_level_publisher = self.create_publisher(Imu, "imu_base", 10)

        ### Services (Clients) ###
        # RGB 
        self.server_set_rgb = self.create_service(SetRGB, "rgb_mode", self.callback_set_rgb) 
        # Torso
        self.server_set_torso_position = self.create_service(SetTorso, "set_torso_position", self.callback_set_torso_position)
        # Motors
        self.activate_motors = self.create_service(ActivateBool, "activate_motors", self.callback_activate_motors)

        self.prev_cmd_vel = Twist()

        self.robot = RobotControl()

        self.robot.set_omni_flags(self.robot.RESET_ENCODERS, True)
        self.robot.set_omni_variables(self.robot.ACCELERATION, 1)
        self.robot.set_omni_flags(self.robot.TIMEOUT, False)
        self.robot.set_omni_variables(self.robot.RGB, 100)

        self.time_cmd_vel = time.time()

        # test and reorganize
        aaa = self.robot.get_omni_variables(self.robot.ACCELERATION)
        # print(aaa)
        
        if aaa[0] == 100:
            self.get_logger().warning(f"Motors are not being powered!")
        else:
            self.get_logger().info(f"Connected to Motor Boards! Accel Ramp Lvl = {aaa[0]}")

        self.create_timer(0.05, self.timer_callback)

    
    def timer_callback(self):

        self.robot.check_data_stream()

        if self.robot.first_data_stream:
            
            data_stream = self.robot.data_stream

            # errors
            errors = ErrorsMotorBoard()
            errors.undervoltage_error = bool(data_stream[0]>>7 & 1 | data_stream[1]>>7 & 1)
            errors.overvoltage_error =  bool(data_stream[0]>>6 & 1 | data_stream[1]>>6 & 1)
            errors.undervoltage_b1_error = bool(data_stream[0]>>7 & 1)
            errors.overvoltage_b1_error = bool(data_stream[0]>>6 & 1)
            errors.undervoltage_b2_error = bool(data_stream[1]>>7 & 1)
            errors.overvoltage_b2_error = bool(data_stream[1]>>6 & 1)
            errors.motor2_b1_short_error = bool(data_stream[0]>>5 & 1)
            errors.motor2_b1_trip_error = bool(data_stream[0]>>4 & 1)
            errors.motor1_b1_short_error = bool(data_stream[0]>>3 & 1)
            errors.motor1_b1_trip_error = bool(data_stream[0]>>2 & 1)
            errors.motor2_b2_short_error = bool(data_stream[1]>>5 & 1)
            errors.motor2_b2_trip_error = bool(data_stream[1]>>4 & 1)
            errors.motor1_b2_short_error = bool(data_stream[1]>>3 & 1)
            errors.motor1_b2_trip_error = bool(data_stream[1]>>2 & 1)
            print("Errors:", errors.undervoltage_error, errors.overvoltage_error)
            self.errors_low_level_publisher.publish(errors)
            # when the emergency stop is pressed it initially says the robot is undervoltage and half a second later changes to overvoltage
            # it is something that comes from the board... it is not a real error

            # encoders
            encoders = Odometry()
            # 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17 

            # buttons
            buttons = ButtonsLowLevel()
            buttons.debug_button1 = bool((data_stream[18] >> 0) & 1)
            buttons.debug_button2 = bool((data_stream[18] >> 1) & 1)
            buttons.debug_button3 = bool((data_stream[18] >> 2) & 1)
            buttons.start_button  = bool((data_stream[18] >> 3) & 1)
            print("Buttons:", buttons.debug_button1, buttons.debug_button2, buttons.debug_button3, buttons.start_button)
            self.buttons_low_level_publisher.publish(buttons)

            # vccs
            vccs = VCCsLowLevel()
            vccs.battery_voltage = ((data_stream[19]/10)*2)+1.0
            vccs.emergency_stop = bool(data_stream[20])
            print("VCCS:", vccs.battery_voltage, vccs.emergency_stop)
            self.vccs_low_level_publisher.publish(vccs)

            # torso
            torso = TorsoPosition()
            torso.legs_position = float(data_stream[21])
            torso.torso_position = float(data_stream[22])
            print("Torso:", torso.legs_position, torso.torso_position)
            self.torso_low_level_publisher.publish(torso)

            # orientation
            orientation = Float32()
            orientation.data = (data_stream[23]<<8|data_stream[24])/10
            orientation.data = 90.0 - orientation.data  # Convert to ROS right-handed frame
            if orientation.data < 0:
                orientation.data += 360.0
            elif orientation.data >= 360:
                orientation.data -= 360.0
            print("Orientation:", orientation.data)
            self.orientation_low_level_publisher.publish(orientation)
            
            # imu
            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = "imu_link"
            imu.angular_velocity.z = float(data_stream[25]<<8|data_stream[26]) 
            if imu.angular_velocity.z >= 32768:  # Convert to signed value
                imu.angular_velocity.z -= 65536
            imu.angular_velocity.z = imu.angular_velocity.z * math.pi / 180.0 # Convert from °/s to rad/s
            imu.angular_velocity_covariance = [-1.0,  0.0,    0.0,  
                                                0.0, -1.0,    0.0,
                                                0.0,  0.0, 1.2e-7]  # Only yaw rate is used
            print("Imu (GyroZ):", imu.angular_velocity.z)

            orientation.data = (data_stream[23]<<8|data_stream[24])/10
            orientation.data = 90.0 - orientation.data  # Convert to ROS right-handed frame
            if orientation.data > 180:
                orientation.data -= 360
            elif orientation.data < -180:
                orientation.data += 360
            orientation.data = math.radians(orientation.data)  # Convert from ° to rad
            q_x, q_y, q_z, q_w = self.get_quaternion_from_euler(0.0, 0.0, yaw=orientation.data)
            imu.orientation.x = q_x
            imu.orientation.y = q_y
            imu.orientation.z = q_z
            imu.orientation.w = q_w
            # Covariance for orientation
            imu.orientation_covariance = [  0.000685,   0.0,        0.0,
                                            0.0,        0.000685,   0.0,
                                            0.0,        0.0,        0.000685]  # Based on 1.5° heading accuracy
            
            self.imu_base_low_level_publisher.publish(imu)

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

    def cmd_vel_callback(self, cmd_vel:Twist):

        # these are just for debug, can delete later:
        # cmd_vel.linear.y = 0.2
        # cmd_vel.linear.x = -cmd_vel.linear.x
        
        t = time.time() - self.time_cmd_vel
        print(round(t,2))
        if t < 0.1:
            return
        
        different_movement = True
        # if cmd_vel.linear.x != self.prev_cmd_vel.linear.x or \
        #    cmd_vel.linear.y != self.prev_cmd_vel.linear.y or \
        #    cmd_vel.angular.z != self.prev_cmd_vel.angular.z:
        #    different_movement= True

        self.prev_cmd_vel = cmd_vel

        # ANGULAR SPEED LINEAR REGRESSION CONVERSION:
        # y = mx+b # however b in this case is 0, so we will only consider y=mx
        angular_speed_m = 59.1965386210101 # calculated via real charmie tests to know the rad/s speeds of the robot
        omni_move_angular_speed = 100 - angular_speed_m*cmd_vel.angular.z
        omni_move_angular_speed = max(0.0, min(200.0, omni_move_angular_speed)) # defines limits for the omni_values
        
        # LINEAR SPEED LINEAR REGRESSION CONVERSION:
        # y = mx+b # however b in this case is 0, so we will only consider y=mx
        linear_speed_m = 237.6182739989   # calculated via real charmie tests to know the rad/s speeds of the robot
        
        # combining x and y linear speeds from /cmd_vel
        combined_linear_speed_xy = math.sqrt(cmd_vel.linear.x*cmd_vel.linear.x + cmd_vel.linear.y*cmd_vel.linear.y) 
        omni_move_linear_speed = linear_speed_m*combined_linear_speed_xy
        omni_move_linear_speed = max(0.0, min(100.0, omni_move_linear_speed)) # defines limits for the omni_values
        
        # total angle combining x and y linear speeds from /cmd_vel
        omni_move_direction = math.atan2(cmd_vel.linear.y, cmd_vel.linear.x) 
        omni_move_direction = math.degrees(omni_move_direction)
        
        while omni_move_direction < 0:
            omni_move_direction += 360
        while omni_move_direction >= 360:
            omni_move_direction -= 360
        # omni_move_direction = max(0.0, min(359.0, omni_move_direction)) # defines limits for the omni_values
        
        print("Diff Lin Vel (x, y):", round(cmd_vel.linear.x, 3), round(cmd_vel.linear.y, 3), "Ang Vel (z):", round(cmd_vel.angular.z, 3))
        
        if different_movement:
            self.robot.omni_move(dir_= int(omni_move_direction), lin_= int(omni_move_linear_speed), ang_= int(omni_move_angular_speed))
            print("Omni move:", "Ang:", int(omni_move_angular_speed), "Lin:", int(omni_move_linear_speed), "Dir:", int(omni_move_direction))
            self.time_cmd_vel = time.time()

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
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
		Convert an Euler angle to a quaternion.
		
		Input
			:param roll: The roll (rotation around x-axis) angle in radians.
			:param pitch: The pitch (rotation around y-axis) angle in radians.
			:param yaw: The yaw (rotation around z-axis) angle in radians.
		
		Output
			:return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
		"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
		
        #print(qx,qy,qz,qw)
  
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = LowLevelNode()
    rclpy.spin(node)
    rclpy.shutdown()
