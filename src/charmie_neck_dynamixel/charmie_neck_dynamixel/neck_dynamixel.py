#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D 
from example_interfaces.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from charmie_interfaces.msg import TrackObject, TrackPerson, DetectedObject, DetectedPerson
from charmie_interfaces.srv import SetNeckPosition, GetNeckPosition

import math
import tty
import termios
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError

DEBUG_DRAW = True

print(os.name)

if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios

    def getch():
        try:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)

            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        except termios.error as e:
            if e.args[0] == 25:
                ch = input("Fallback input: ")
            else:  
                raise
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *  # Uses Dynamixel SDK library

##########      PAN (ID = 1)  bottom servo (left and right)
##########      TILT (ID = 2)     up servo (up and down)

# Control table address
ADDR_MX_TORQUE_ENABLE = 24  # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_PRESENT_POSITION = 36

ADDR_MX_D_GAIN = 26  # Control table address is different in Dynamixel model
ADDR_MX_I_GAIN = 27  # Control table address is different in Dynamixel model
ADDR_MX_P_GAIN = 28  # Control table address is different in Dynamixel model

# Different PID gains for each axis
PAN_D_GAIN = 1
PAN_I_GAIN = 1
PAN_P_GAIN = 3 # 6

# Different PID gains for each axis
TILT_D_GAIN = 2
TILT_I_GAIN = 5
TILT_P_GAIN = 6

# Protocol version
PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_PAN = 1  # Dynamixel ID : 1 
DXL_ID_TILT = 2  # Dynamixel ID : 2
BAUDRATE = 57600  # Dynamixel default baudrate : 57600
# MAC GIL # DEVICENAME = '/dev/tty.usbserial-AI0282RX'  # Check which port is being used on your controller
DEVICENAME = '/dev/ttyUSB1'  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
# DXL_MINIMUM_POSITION_VALUE = 700  # Dynamixel will rotate between this value
# DXL_MAXIMUM_POSITION_VALUE = 3200  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
# DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

DEGREES_TO_SERVO_TICKS_CONST = 11.3777292
SERVO_TICKS_TO_DEGREES_CONST = 0.087891

# max angles for each servo to prevent incorrect values
MAX_PAN_ANGLE = 359
MIN_PAN_ANGLE = 0
MAX_TILT_ANGLE = 235
MIN_TILT_ANGLE = 120

pan = 2048
tilt = 2048

# There is two ways of reading the servo position:
# The closed loop way is by sending a request to the servos and waiting for the return of the encoder position (closed loop)
# However this is only a good approach when after a while (0.5 sec) the servos stopped moving, otherwise we receive a lot of noise.
# Therefore in the current code this is only used for the initial reading of the servo position.
# The second way is just to save the variables sent to the servos (open loop). Due to the high quality of the servos the eror is never higher than 0.5 degrees
# The second way has some slight error however it is more reliable than the closed loop. 
# The initial values on these variables are the servo values when it is in rest position (after turning on the system): (180, 115)
read_pan_closed_loop = 180*DEGREES_TO_SERVO_TICKS_CONST
read_tilt_closed_loop = 115*DEGREES_TO_SERVO_TICKS_CONST
read_pan_open_loop = 180*DEGREES_TO_SERVO_TICKS_CONST
read_tilt_open_loop = 115*DEGREES_TO_SERVO_TICKS_CONST

# The servos start position (180, 180) does not make the robot look forward because of the camera and face angle.
# These values are necessary to adjust to this position shift 
# With recent changes to hardware, the neck part is now perpendicular to the floor, therefore this is no longer necessary
PAN_CONST_SHIFT = 0
TILT_CONST_SHIFT = 0  # (180 + 10 = 190)

# index = 0
# dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]  # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)


class NeckNode(Node):

    def __init__(self):
        super().__init__("Neck")
        self.get_logger().info("Initialised CHARMIE Neck Node")
        print("Connected to Neck Board via:", DEVICENAME)  # check which port was really used

        # receives two angles, pan and tilt - used when robot must look at something known in advance (ex: direction of navigation, forward, look right/left)
        ########### self.neck_position_subscriber = self.create_subscription(NeckPosition, "neck_to_pos", self.neck_position_callback ,10)
        # receives coordinates where the robot must look at, knowing its own position (ex: look at the couch, look at the table)
        self.neck_to_coords_subscriber = self.create_subscription(Pose2D, "neck_to_coords", self.neck_to_coords_callback, 10)

        # receives a person and the keypoint it must follow (ex: constantly looking at the person face, look at body center  to check hands and feet)
        self.neck_follow_person_subscriber = self.create_subscription(TrackPerson, "neck_follow_person", self.neck_follow_person_callback ,10)
        # receives an object and it follows it, keeping it centered in the image (ex: constantly looking at a cup, plate, cereal box)
        self.neck_follow_object_subscriber = self.create_subscription(TrackObject, "neck_follow_object", self.neck_follow_object_callback, 10)

        # sends the current position of the servos after every change made on the publisher topics
        ########### self.neck_get_position_publisher = self.create_publisher(NeckPosition, "get_neck_pos", 10)

        # subscribes to robot position, to allow neck_to_coords
        self.odom_subscriber = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        # standard diagnostic publisher
        self.neck_diagnostic_publisher = self.create_publisher(Bool, "neck_diagnostic", 10)


        # SERVICES:
        # Main receive commads 
        self.server_neck_position = self.create_service(SetNeckPosition, "neck_to_pos", self.callback_set_neck_position) 
        self.server_neck_position = self.create_service(GetNeckPosition, "get_neck_pos", self.callback_get_neck_position) 
        self.get_logger().info("Neck Servers have been started")


        # if DEBUG_DRAW:
        #     self.img = Image()
        #     self.first_img_ready = False
        #     self.color_image_subscriber = self.create_subscription(Image, "/color/image_raw", self.get_color_image_callback, 10)
            

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_t = 0.0
         
        self.flag_get_neck_position = False

        self.initialise_servos()

        flag_diagn = Bool()
        flag_diagn.data = True
        self.neck_diagnostic_publisher.publish(flag_diagn)
        


    ########## SERVICES ##########
    def callback_set_neck_position(self, request, response):
        
        # Type of service received: 
        # float64 pan  # value to bottom servo, rotate left ot right
        # float64 tilt # value to top servo, rotate up or down
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        self.get_logger().info("Received Neck Position %s" %("("+str(request.pan)+", "+str(request.tilt)+")"))
        # print("Received Position: pan =", coords.x, " tilt = ", coords.y)
        
        # +180.0 on both values since for calculations (180, 180) is the middle position but is easier UI for center to be (0,0)
        self.move_neck(request.pan+180.0, request.tilt+180.0)

        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "neck test"
        return response

    def callback_get_neck_position(self, request, response):
        
        # Type of service received: 
        # (nothing)
        # ---
        # float64 pan  # value to bottom servo, rotate left ot right
        # float64 tilt # value to top servo, rotate up or down

        self.get_logger().info("Received Get Neck Position")
        
        # returns the reading values of both neck servos
        response.pan  = float(int(read_pan_open_loop *SERVO_TICKS_TO_DEGREES_CONST + 0.5)) - 180.0
        response.tilt = float(int(read_tilt_open_loop*SERVO_TICKS_TO_DEGREES_CONST + 0.5)) - 180.0

        return response


    ########## CALLBACKS ##########
    # def neck_position_callback(self, neck_pos: NeckPosition):
    #     self.get_logger().info("Received Neck Position, Adjusting the Neck Position")
    #     # print("Received Position: pan =", coords.x, " tilt = ", coords.y)
    #     
    #     # +180.0 on both values since for calculations (180, 180) is the middle position but is easier UI for center to be (0,0)
    #     self.move_neck(neck_pos.pan+180.0, neck_pos.tilt+180.0)


    def neck_to_coords_callback(self, pose: Pose2D):
        # calculate the angle according to last received odometry
        neck_target_x = pose.x
        neck_target_y = pose.y
        neck_target_other_axis = pose.theta

        ##### remove TILT_CONST_SHIFT #####
        
        global TILT_CONST_SHIFT # in this case it only makes sense to change the tilt because the pan is abstract   

        # print(math.degrees(self.robot_t))
        
        ang = math.atan2(self.robot_y - neck_target_y, self.robot_x - neck_target_x) + math.pi/2
        # print("ang_rad:", ang)
        ang = math.degrees(ang)
        # print("ang_deg:", ang)

        pan_neck_to_coords = math.degrees(self.robot_t) - ang
        if pan_neck_to_coords < -math.degrees(math.pi):
            pan_neck_to_coords += math.degrees(2*math.pi)

        print("neck_to_coords:", pan_neck_to_coords, ang)
        self.move_neck(180 - pan_neck_to_coords, neck_target_other_axis + TILT_CONST_SHIFT)


    def odom_callback(self, odom: Odometry):
        # update the last odom value
                
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y

        qx = odom.pose.pose.orientation.x
        qy = odom.pose.pose.orientation.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w

        self.robot_t = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        
        # print(self.robot_x, self.robot_y, self.robot_t)


    def neck_follow_person_callback(self, pose: TrackPerson):
        print("Folow person received")

        global read_pan_open_loop, read_tilt_open_loop


        img_width = 1280
        img_height = 720


        target_x = pose.person.kp_nose_x
        target_y = pose.person.kp_nose_y

        hor_fov = 91.2
        ver_fov = 65.5

        print(target_x, target_y)

        error_x = -int(img_width/2 - pose.person.kp_nose_x)
        error_y = -int(img_height/2 - pose.person.kp_nose_y)

        perc_x = error_x/(img_width/2)
        perc_y = error_y/(img_height/2)

        new_a_x = (-perc_x*(hor_fov/2))
        new_a_y = (-perc_y*(ver_fov/2))*0.75 # on the 'yes movement' axis, it tended to always overshoot a bit, the 0.75 factor fixes it

        print("angs: ", new_a_x, new_a_y)

        # print(read_pan_open_loop*SERVO_TICKS_TO_DEGREES_CONST + new_a_x, read_tilt_open_loop*SERVO_TICKS_TO_DEGREES_CONST + new_a_y)
        self.send_neck_move(read_pan_open_loop*SERVO_TICKS_TO_DEGREES_CONST + new_a_x, read_tilt_open_loop*SERVO_TICKS_TO_DEGREES_CONST + new_a_y)
        
        # if DEBUG_DRAW and self.first_img_ready:
        #     br = CvBridge()
        #     current_frame = br.imgmsg_to_cv2(self.img, "bgr8")
        # 
        #     # cv2.line(current_frame, (img_width//2, img_height//2), (img_width//2+error_x, img_height//2+error_y), (255,0,0), 3)
        #     cv2.line(current_frame, (img_width//2, img_height//2), (img_width//2, img_height//2+error_y), (255,0,0), 3)
        #     cv2.line(current_frame, (img_width//2, img_height//2), (img_width//2+error_x, img_height//2), (255,0,0), 3)
        # 
        #     cv2.circle(current_frame, (img_width//2, img_height//2), 3, (0,0,255), -1)
        #     cv2.circle(current_frame, (target_x, target_y), 3, (0,0,255), -1)
        # 
        #     cv2.imshow("Neck Debug", current_frame)
        #     cv2.waitKey(1)

    def neck_follow_object_callback(self, pose: TrackObject):
        pass

    # def get_color_image_callback(self, img: Image):
        # self.get_logger().info('Receiving color video frame')
        # self.img = img
        # self.first_img_ready = True


    ########## NECK CNOTROL FUNCTIONS ##########
    # Initially I created a class for NeckControl, however due to the errors in readings of neck positios from the servos, we changed to open loop readings.
    # This means that, due to the high quality of servos, if I give a value of 180 degrees, the maximum error I get is 0.5 degrees.
    # So the open loop readings are quite trustfull. However if I want to publish in the get_neck_pos topic, everytime i send a vlue to the servos, I cannot 
    # do it in a different class that neck node. So for CHARMIE neck we will only be using NeckNode class.    
    def initialise_servos(self):

        global read_pan_open_loop, read_tilt_open_loop
        
        # Open port
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("TILT %s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("TILT %s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel TILT has been successfully connected")

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("PAN %s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("PAN %s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel PAN has been successfully connected")
        # self.get_logger().info("Set Torque Mode")

        # Set PID parameters for each servo (different params because of how smooth the movement must be on each axis)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_D_GAIN, PAN_D_GAIN)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_I_GAIN, PAN_I_GAIN)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_P_GAIN, PAN_P_GAIN)

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_D_GAIN, TILT_D_GAIN)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_I_GAIN, TILT_I_GAIN)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_P_GAIN, TILT_P_GAIN)

        read_pan_open_loop, read_tilt_open_loop = self.read_servo_position()

        self.get_logger().info("Set Neck to Initial Position, Looking Forward")

        self.move_neck(180, 180) # resets the neck whenever the node is started, so that at the beginning the neck is always facing forward 
        
        # self.move_neck(180, 180) # resets the neck whenever the node is started, so that at the beginning the neck is always facing forward 
        # time.sleep(3)

    def read_servo_position(self):

        global read_pan_closed_loop, read_tilt_closed_loop

        p, dxl_comm_result, dxl_error  = packetHandler.read4ByteTxRx(portHandler, DXL_ID_PAN,  ADDR_MX_PRESENT_POSITION)
        t, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_PRESENT_POSITION)
        
        # when the servo is moving it usually returns values extremely high, this way these values are filtered
        if p < 4096:
            read_pan_closed_loop = p
        if t < 4096:
            read_tilt_closed_loop = t
        
        print("read (closed loop):", read_pan_closed_loop*SERVO_TICKS_TO_DEGREES_CONST, read_tilt_closed_loop*SERVO_TICKS_TO_DEGREES_CONST)

        ########## self.publish_get_neck_pos(read_pan_closed_loop, read_tilt_closed_loop)

        return read_pan_closed_loop, read_tilt_closed_loop


    # def publish_get_neck_pos(self, p, t):
    # 
    #     pose = NeckPosition()
    #     pose.pan  = float(int(p*SERVO_TICKS_TO_DEGREES_CONST + 0.5))
    #     pose.tilt = float(int(t*SERVO_TICKS_TO_DEGREES_CONST + 0.5))
    #     self.neck_get_position_publisher.publish(pose)


    def move_neck(self, p, t):
        global read_pan_open_loop, read_tilt_open_loop

        p = int(p)
        t = int(t)

        print("START")
        
        d_t = 0.02
        u_pan = 10
        u_tilt_up = 5 #300 # 5
        u_tilt_down = 2 #300 # 2

        signal_pan = 1
        signal_tilt = 1

        read_pan_open_loop_deg = int(read_pan_open_loop*SERVO_TICKS_TO_DEGREES_CONST)
        read_tilt_open_loop_deg = int(read_tilt_open_loop*SERVO_TICKS_TO_DEGREES_CONST)

        pan_dif = int(p) - read_pan_open_loop_deg
        
        if pan_dif < 0.0:
            signal_pan = -1
        else:
            signal_pan = 1

        div_pan = int(abs(pan_dif)//u_pan)*signal_pan
        rem_pan = int(abs(pan_dif)%u_pan)*signal_pan


        tilt_dif = int(t) - read_tilt_open_loop_deg

        if tilt_dif < 0.0:
            signal_tilt = -1
            u_tilt =  u_tilt_down
        else:
            signal_tilt = 1
            u_tilt = u_tilt_up

        div_tilt = int(abs(tilt_dif)//u_tilt)*signal_tilt
        rem_tilt = int(abs(tilt_dif)%u_tilt)*signal_tilt

        
        print("PAN:", read_pan_open_loop_deg, " -> ", p, "dif:", pan_dif, "u_pan:", u_pan, "div:", div_pan, "rem:", rem_pan)
        print("TILT:", read_tilt_open_loop_deg, " -> ", t, "dif:", tilt_dif, "u_tilt:", u_tilt, "div:", div_tilt, "rem:", rem_tilt)

        ctr = 1
        new_tilt = read_tilt_open_loop_deg+rem_tilt
        new_pan = read_pan_open_loop_deg+rem_pan
        self.send_neck_move(new_pan, new_tilt) 
        while(t != new_tilt or p!=new_pan):
            time.sleep(d_t)
            
            if new_pan == p:
                new_pan = p
            else:
                new_pan = read_pan_open_loop_deg+rem_pan+((ctr)*signal_pan*u_pan)
            
            if new_tilt == t:
                new_tilt = t
            else:
                new_tilt = read_tilt_open_loop_deg+rem_tilt+((ctr)*signal_tilt*u_tilt)
            
            self.send_neck_move(new_pan, new_tilt) # resets the neck whenever the node is started, so that at the beginning the neck is always facing forward 
            ctr+=1
        print("ctr:", ctr)
        
        print("END")


    def send_neck_move(self, p, t):

        global read_pan_open_loop, read_tilt_open_loop

        if p > MAX_PAN_ANGLE:
            p = MAX_PAN_ANGLE
        if p < MIN_PAN_ANGLE:
            p = MIN_PAN_ANGLE

        if t > MAX_TILT_ANGLE:
            t = MAX_TILT_ANGLE
        if t < MIN_TILT_ANGLE:
            t = MIN_TILT_ANGLE
        
        print(p,t)

        p = int(p * DEGREES_TO_SERVO_TICKS_CONST + 0.5)
        t = int(t * DEGREES_TO_SERVO_TICKS_CONST + 0.5)

        read_pan_open_loop = p
        read_tilt_open_loop = t  

        ########## self.publish_get_neck_pos(read_pan_open_loop, read_tilt_open_loop)
        
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_GOAL_POSITION, p)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_GOAL_POSITION, t)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))


def main(args=None):
    rclpy.init(args=args)
    node = NeckNode()
    rclpy.spin(node)
    rclpy.shutdown()