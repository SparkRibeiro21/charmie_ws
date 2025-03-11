#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Pose2D, Point, TransformStamped
from example_interfaces.msg import Bool
from charmie_interfaces.msg import NeckPosition
from charmie_interfaces.srv import SetNeckPosition, GetNeckPosition, SetNeckCoordinates, TrackPerson, TrackObject, TrackContinuous

import math
import tty
import termios
import os
import time

# function to calculate zeros of non linear functions. In this case: to calculate the tilt (up/down) angle to look at coordinates
from scipy.optimize import fsolve

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
PAN_D_GAIN = 2
PAN_I_GAIN = 2
PAN_P_GAIN = 6

# Different PID gains for each axis
TILT_D_GAIN = 4
TILT_I_GAIN = 10
TILT_P_GAIN = 12

# Protocol version
PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_PAN = 1  # Dynamixel ID : 1 
DXL_ID_TILT = 2  # Dynamixel ID : 2
BAUDRATE = 57600  # Dynamixel default baudrate : 576

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
MAX_TILT_ANGLE = 240
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

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)


class NeckNode(Node):

    def __init__(self):
        super().__init__("Neck")
        self.get_logger().info("Initialised CHARMIE Neck Node")

        self.declare_parameter("speed_up", 3) 
        self.declare_parameter("speed_down", 2) 
        self.declare_parameter("speed_sides", 5)
        self.declare_parameter("initial_position", [0, 0])  # Default value
        # example of initial_position param: --ros-args -p initial_position:="[0, 10]"

        # CONTROL VARIABLES
        self.u_tilt_up = self.get_parameter("speed_up").value
        self.u_tilt_down = self.get_parameter("speed_down").value
        self.u_pan = self.get_parameter("speed_sides").value
        self.initial_position = self.get_parameter("initial_position").value
        
        # open serial port by id, this removes problems with volative port names: "/dev/ttyUSBX"
        DEVICENAME = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI0282RX-if00-port0"
        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        ########## CHANGE TO LOGGER ##########
        print("Connected to Neck Board via:", DEVICENAME)  # check which port was really used
        
        self.robot_pose = Pose2D()

        self.continuous_tracking = False
        self.continuous_tracking_point_position = Point()

        self.tracking_target_p = 0
        self.tracking_target_t = 0
        self.tracking_ctr = 1
        self.tracking_new_pan = 0
        self.tracking_new_tilt = 0
        self.tracking_read_pan_open_loop_deg = 0
        self.tracking_read_tilt_open_loop_deg = 0
        self.tracking_signal_pan = 0
        self.tracking_signal_tilt = 0
        self.tracking_u_pan = 0
        self.tracking_u_tilt = 0
        self.tracking_rem_pan = 0
        self.tracking_rem_tilt = 0

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # TOPICS:
        # sends the current position of the servos after every change made on the publisher topics
        self.neck_get_position_topic_publisher = self.create_publisher(NeckPosition, "get_neck_pos_topic", 10)
        # Robot Localisation
        self.robot_localisation_subscriber = self.create_subscription(Pose2D, "robot_localisation", self.robot_localisation_callback, 10)
        # Continuous Tracking
        self.continuous_tracking_position_subscriber = self.create_subscription(Point, "continuous_tracking_position", self.continuous_tracking_position_callback, 10)
        
        self.initialise_servos()

        # SERVICES:
        # Main receive commads 
        self.server_set_neck_position = self.create_service(SetNeckPosition, "neck_to_pos", self.callback_set_neck_position) 
        self.server_get_neck_position = self.create_service(GetNeckPosition, "get_neck_pos", self.callback_get_neck_position) 
        self.server_set_neck_to_coordinates = self.create_service(SetNeckCoordinates, "neck_to_coords", self.callback_set_neck_to_coordinates) 
        self.server_neck_track_person = self.create_service(TrackPerson, "neck_track_person", self.callback_neck_track_person)
        self.server_neck_track_object = self.create_service(TrackObject, "neck_track_object", self.callback_neck_track_object)
        self.server_neck_continuous_tracking = self.create_service(TrackContinuous, "set_continuous_tracking", self.callback_continuous_tracking)
        self.get_logger().info("Neck Servers have been started")

        # timer that checks the controller every 50 ms 
        self.create_timer(1.0, self.timer_callback_neck_position)
        self.create_timer(0.025, self.timer_callback_continuous_tracking)


    ########## TIMER ##########
    def timer_callback_neck_position(self):

        # if any of the modules that need neck info (calculate positions of any detected object/person)
        # is turned on after the initial neck movement and no other neck movement is made, these variables are never updated
        # this way, these are updated periodically
        global read_pan_open_loop, read_tilt_open_loop

        self.publish_get_neck_pos(read_pan_open_loop, read_tilt_open_loop)


    def timer_callback_continuous_tracking(self):
        
        if self.continuous_tracking:

            error_tilt = abs(self.tracking_target_t - self.tracking_new_tilt)
            error_pan  = abs(self.tracking_target_p - self.tracking_new_pan) 

            max_error_p = 7
            max_error_t = 7

            print("Tr_PAN:", self.tracking_target_p, " -> ", self.tracking_target_p, "error_p:", error_pan)
            print("Tr_TILT:", self.tracking_target_t, " -> ", self.tracking_target_t, "error_t:", error_tilt)


            if error_pan < 20:# 
                self.tracking_u_pan = 1
            elif error_pan < 30:
                self.tracking_u_pan = 2
            elif error_pan < 40:
                self.tracking_u_pan = 3

            print("pan_speed:", self.tracking_u_pan)
            
            if(error_tilt>max_error_t or error_pan>max_error_p):
                
                if error_pan<=max_error_p:
                    pass # keeps the same value
                    # self.tracking_new_pan = self.tracking_target_p
                    print("P_S_E")
                else:
                    self.tracking_new_pan = self.tracking_read_pan_open_loop_deg+self.tracking_rem_pan+((self.tracking_ctr)*self.tracking_signal_pan*self.tracking_u_pan)
                    print("P_C_E")

                if error_tilt<=max_error_t:
                    pass # keeps the same value
                    # self.tracking_new_tilt = self.tracking_target_t
                    print("T_S_E")
                else:
                    self.tracking_new_tilt = self.tracking_read_tilt_open_loop_deg+self.tracking_rem_tilt+((self.tracking_ctr)*self.tracking_signal_tilt*self.tracking_u_tilt)
                    print("T_C_E")
                
                self.send_neck_move(self.tracking_new_pan, self.tracking_new_tilt)
                self.tracking_ctr+=1
            

    def continuous_tracking_position_callback(self, position: Point):
        self.continuous_tracking_point_position = position
        self.move_neck_with_target_pixel(target_x=self.continuous_tracking_point_position.x, target_y=self.continuous_tracking_point_position.y, tracking_mode=True)
        print("NEW")

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
        response.message = "Set Neck Position"
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

    def callback_set_neck_to_coordinates(self, request, response):
        
        # Type of service received: 
        # geometry_msgs/Point coords  # house coordinates the robot must look at 
        # bool is_tilt # if this flag is true, the robot will use the tilt value for the tilt servo rather than the z value of coords
        # float64 tilt # ang value to be set on tilt servo
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        # calculate the angle according to last received odometry
        target_x = request.coords.x
        target_y = request.coords.y
        target_z = request.coords.z

        self.get_logger().info("Received Neck Coordinates %s" %("("+str(target_x)+", "+str(target_y)+", "+str(target_z)+")"))
        
        self.move_neck_with_target_coordinates(target_x=target_x, target_y=target_y, target_z=target_z, tracking_mode=False)
        
        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Set Neck to (x,y,z) Coordinates"
        return response

    def callback_neck_track_person(self, request, response):

        # Type of service received: 
        # DetectedPerson person # The person it is intended to be followed by the neck
        # string body_part # body part the robot must look at
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        # these are the only two points that will always exist independentely if the confidence of a keypoint is > MIN_CONFIDENCE_VALUE
        if request.body_part == "Head":
            target_x = request.person.head_center_x
            target_y = request.person.head_center_y
        elif request.body_part == "Torso":
            target_x = request.person.body_center_x
            target_y = request.person.body_center_y
            
        self.get_logger().info("Received Neck Track Person %s" %("("+str(target_x)+", "+str(target_y)+")"))
        
        self.move_neck_with_target_pixel(target_x=target_x, target_y=target_y, tracking_mode=False)

        response.success = True
        response.message = "Neck Track Person"
        return response

    
    def callback_neck_track_object(self, request, response):

        # Type of service received: 
        # DetectedObject object # The object it is intended to be followed by the neck
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        target_x = request.object.box_center_x
        target_y = request.object.box_center_y

        self.get_logger().info("Received Neck Track Object %s" %("("+str(target_x)+", "+str(target_y)+")"))
        
        self.move_neck_with_target_pixel(target_x=target_x, target_y=target_y)

        response.success = True
        response.message = "Neck Track Object"
        return response
    
    def callback_continuous_tracking(self, request, response):
        
        # Type of service received: 
        # bool status # turns on anf off the continuous tracking mode
        # string tracking_type # select the type of continuous tracking
        # geometry_msgs/Point tracking_position # 3D coordinates of continuous tracking subject 
        # ---
        # bool success   # indicate successful run of triggered service
        # string message # informational, e.g. for error messages.

        self.get_logger().info("Received Neck Continuous Tracking: %s" %str(request.status))

        self.continuous_tracking = request.status
        
        # returns whether the message was played and some informations regarding status
        response.success = True
        response.message = "Set Neck Position"
        return response


    ########## CALLBACKS ##########
    def robot_localisation_callback(self, pose: Pose2D):
        self.robot_pose = pose

    ########## NECK CONTROL FUNCTIONS ##########
    # Initially I created a class for NeckControl, however due to the errors in readings of neck positios from the servos, we changed to open loop readings.
    # This means that, due to the high quality of servos, if I give a value of 180 degrees, the maximum error I get is 0.5 degrees.
    # So the open loop readings are quite trustfull. However if I want to publish in the get_neck_pos topic, everytime i send a vlue to the servos, I cannot 
    # do it in a different class that neck node. So for CHARMIE neck we will only be using NeckNode class.    
    def initialise_servos(self):

        global read_pan_open_loop, read_tilt_open_loop
        
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        comms_with_neck_servo_established = False
        while not comms_with_neck_servo_established:

            comms_pan = False
            comms_tilt = False

            # Enable Dynamixel Torque
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_TILT, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("TILT %s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("TILT %s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel TILT has been successfully connected")
                comms_tilt = True

            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_PAN, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("PAN %s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("PAN %s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel PAN has been successfully connected")
                comms_pan = True

            if comms_tilt and comms_pan:
                comms_with_neck_servo_established = True
            else:
                self.get_logger().error("Neck Module not Powered!")
                time.sleep(0.2)
                
        # Set PID parameters for each servo (different params because of how smooth the movement must be on each axis)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_PAN, ADDR_MX_D_GAIN, PAN_D_GAIN)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_PAN, ADDR_MX_I_GAIN, PAN_I_GAIN)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_PAN, ADDR_MX_P_GAIN, PAN_P_GAIN)

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_TILT, ADDR_MX_D_GAIN, TILT_D_GAIN)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_TILT, ADDR_MX_I_GAIN, TILT_I_GAIN)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_TILT, ADDR_MX_P_GAIN, TILT_P_GAIN)

        read_pan_open_loop, read_tilt_open_loop = self.read_servo_position()

        self.get_logger().info("Set Neck to Initial Position, Looking Forward")

        # changed to two different positions so that in any case it is visible the neck moving when is started 
        self.move_neck(180, 135) # resets the neck whenever the node is started, so that at the beginning the neck is always facing forward 
        self.move_neck(180+self.initial_position[0], 180+self.initial_position[1]) # resets the neck whenever the node is started, so that at the beginning the neck is always facing forward 
        # self.move_neck(180, 180) # resets the neck whenever the node is started, so that at the beginning the neck is always facing forward 
        
        
    def read_servo_position(self):

        global read_pan_closed_loop, read_tilt_closed_loop

        p, dxl_comm_result, dxl_error  = packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_PAN,  ADDR_MX_PRESENT_POSITION)
        t, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_TILT, ADDR_MX_PRESENT_POSITION)
        
        # when the servo is moving it usually returns values extremely high, this way these values are filtered
        if p < 4096:
            read_pan_closed_loop = p
        if t < 4096:
            read_tilt_closed_loop = t
        
        print("read (closed loop):", read_pan_closed_loop*SERVO_TICKS_TO_DEGREES_CONST, read_tilt_closed_loop*SERVO_TICKS_TO_DEGREES_CONST)

        # self.publish_get_neck_pos(read_pan_closed_loop, read_tilt_closed_loop)

        return read_pan_closed_loop, read_tilt_closed_loop


    def publish_get_neck_pos(self, p, t):

        # this function is used for nodes that need to keep the latest neck position value saved, so when new data comes
        # these need to compute, they don't have to request the position, therefore not wasting time...
        pose = NeckPosition()
        pose.pan  = float(int(p*SERVO_TICKS_TO_DEGREES_CONST + 0.5)) - 180.0
        pose.tilt = float(int(t*SERVO_TICKS_TO_DEGREES_CONST + 0.5)) - 180.0
        ### print(pose) # THIS IS THE COMMENT THAT SHOWS EVERY MOVEMENT ITERATION
        self.neck_get_position_topic_publisher.publish(pose)
        self.publish_neck_tf2s(pose)


    def publish_neck_tf2s(self, pose: NeckPosition):

        # Publish the pan TF of the neck
        neck_pan_transform = TransformStamped()
        # Set the timestamp to the current time
        neck_pan_transform.header.stamp = self.get_clock().now().to_msg()

        # Set the parent and child frames (the link names)
        neck_pan_transform.header.frame_id = 'neck_base_link'  # Parent frame
        neck_pan_transform.child_frame_id = 'neck_pan_link'  # Child frame

        # Set the translation (position) of the neck relative to the base
        neck_pan_transform.transform.translation.x = 0.0  # Example value
        neck_pan_transform.transform.translation.y = 0.0  # Example value
        neck_pan_transform.transform.translation.z = 0.0  # Example value (height)

        deg_pan = math.radians(pose.pan)
        # Set the rotation (orientation) of the neck (in quaternion format)
        neck_pan_transform.transform.rotation.x = 0.0
        neck_pan_transform.transform.rotation.y = 0.0
        neck_pan_transform.transform.rotation.z = math.sin(deg_pan/2.0)
        neck_pan_transform.transform.rotation.w = math.cos(deg_pan/2.0)

        # Publish the transform
        self.tf_broadcaster.sendTransform(neck_pan_transform)
        # print(pose.pan)
        # self.get_logger().info('Published TF from base_link to neck_pan_link')

        # Publish the tilt TF of the neck
        neck_tilt_transform = TransformStamped()
        # Set the timestamp to the current time
        neck_tilt_transform.header.stamp = self.get_clock().now().to_msg()

        # Set the parent and child frames (the link names)
        neck_tilt_transform.header.frame_id = 'neck_pan_link'  # Parent frame
        neck_tilt_transform.child_frame_id = 'neck_tilt_link'  # Child frame

        # Set the translation (position) of the neck relative to the base
        neck_tilt_transform.transform.translation.x = 0.03   # Example value
        neck_tilt_transform.transform.translation.y = 0.0    # Example value
        neck_tilt_transform.transform.translation.z = 0.0250 # Example value (height)

        deg_tilt = -math.radians(pose.tilt)
        # Set the rotation (orientation) of the neck (in quaternion format)
        neck_tilt_transform.transform.rotation.x = 0.0
        neck_tilt_transform.transform.rotation.y = math.sin(deg_tilt/2.0)
        neck_tilt_transform.transform.rotation.z = 0.0
        neck_tilt_transform.transform.rotation.w = math.cos(deg_tilt/2.0)

        # Publish the transform
        self.tf_broadcaster.sendTransform(neck_tilt_transform)
        # print(pose.tilt)
        # self.get_logger().info('Published TF from neck_pan_link to neck_tilt_link')




    def move_neck_with_target_coordinates(self, target_x, target_y, target_z, tracking_mode=False):

        ### PAN MOVEMENT (LEFT - RIGHT)

        # 6.5 cm adjustement from bottom servo to robot center, this helps in cases where angle to coordinates are near 90 and 270 degrees
        # where there was an error of 4/5 degrees because the axis were not alligned 
        bottom_servo_to_robot_center = 0.065  
        ang = math.atan2(self.robot_pose.y - target_y, -(self.robot_pose.x + bottom_servo_to_robot_center - target_x))
        # print("ang_rad:", ang)
        ang = math.degrees(ang)
        # print("ang_deg:", ang)
    
        # print(math.degrees(self.robot_pose.theta))     
        pan_neck_to_coords = - math.degrees(self.robot_pose.theta) - ang
        if pan_neck_to_coords < -math.degrees(math.pi):
            pan_neck_to_coords += math.degrees(2*math.pi)
        if pan_neck_to_coords > math.degrees(math.pi):
            pan_neck_to_coords -= math.degrees(2*math.pi)
    
        print(pan_neck_to_coords)
        # if the robot wants to look back, it uses the correct side to do so without damaging itself
        # this was left here but on practice is not used (decided to comment but left in case it makes sense in the future)
        # if pan_neck_to_coords == -180:
        #     pan_neck_to_coords = 180

        # print("neck_to_coords:", pan_neck_to_coords, ang)

        ### TILT MOVEMENT (UP - DOWN)
        dist = math.sqrt((self.robot_pose.y - target_y)**2 + (self.robot_pose.x - target_x)**2)

        # Constants
        h = 1.30 # height of rotation axis of up/down servo from the ground (should be automatic). Does not consider changes in torso.
        c = 0.06 # distance from center rotation axis of up/down servo to face (horizontal when looking forward)
        d = 0.09 # distance from c to center of face. This way the center of the face is looking at the person and not the camera or servo.
        e = math.sqrt(c**2 + d**2)
        a = target_z
        b = dist
            
        # Define the function based on the equation
        def equation(alpha):
            return alpha - math.atan(c / d) - math.atan((h + e * math.cos(alpha) - a) / (b - e * math.sin(alpha)))
            # return alpha - np.arctan(c / d) - np.arctan((h + e * np.sin(alpha) - a) / (b - e * np.cos(alpha)))

        # Initial guess for alpha
        initial_guess = 0

        # initial_time = time.time() 
        # Solve the equation
        alpha_solution = fsolve(equation, initial_guess)
        # elapsed_time = time.time() - initial_time

        phi = math.atan(d / c)

        final_x = - (math.degrees(alpha_solution[0]) + math.degrees(phi) - 90)

        # Debug prints of calculations of up/down movement:
        # print("Alpha:", math.degrees(alpha_solution[0]))
        # print("Phi:", math.degrees(phi))
        # print("Alpha+Phi:", round(final_x, 2))
        # print("Time:", elapsed_time)

        if final_x > 0.0: # solves rounding errors when variable is positive
            final_x += 0.5

        self.move_neck(180 + pan_neck_to_coords, final_x+180.0, tracking_mode=tracking_mode)

        
    def move_neck_with_target_pixel(self, target_x, target_y, tracking_mode=False):
    
        global read_pan_open_loop, read_tilt_open_loop

        # print(target_x, target_y)

        img_width = 1280
        img_height = 720

        # target_x = request.person.kp_nose_x
        # target_y = request.person.kp_nose_y

        hor_fov = 91.2
        ver_fov = 65.5

        print(target_x, target_y)

        error_x = -int(img_width/2 -  target_x)
        error_y = -int(img_height/2 - target_y)

        perc_x = error_x/(img_width/2)
        perc_y = error_y/(img_height/2)

        new_a_x = (-perc_x*(hor_fov/2))
        new_a_y = (-perc_y*(ver_fov/2))*0.75 # on the 'yes movement' axis, it tended to always overshoot a bit, the 0.75 factor fixes it

        print("angs: ", new_a_x, new_a_y)

        # print(read_pan_open_loop*SERVO_TICKS_TO_DEGREES_CONST + new_a_x, read_tilt_open_loop*SERVO_TICKS_TO_DEGREES_CONST + new_a_y)
        self.move_neck(read_pan_open_loop*SERVO_TICKS_TO_DEGREES_CONST + new_a_x, read_tilt_open_loop*SERVO_TICKS_TO_DEGREES_CONST + new_a_y, tracking_mode=tracking_mode)


    def move_neck(self, p, t, tracking_mode=False):
        global read_pan_open_loop, read_tilt_open_loop

        p = int(p)
        t = int(t)

        # print("START")
        
        d_t = 0.02

        signal_pan = 1
        signal_tilt = 1

        read_pan_open_loop_deg = int(read_pan_open_loop*SERVO_TICKS_TO_DEGREES_CONST)
        read_tilt_open_loop_deg = int(read_tilt_open_loop*SERVO_TICKS_TO_DEGREES_CONST)

        pan_dif = int(p) - read_pan_open_loop_deg
        
        if pan_dif < 0.0:
            signal_pan = -1
        else:
            signal_pan = 1

        if not tracking_mode:
            speed_sides = self.u_pan
        else:
            speed_sides = 2


        div_pan = int(abs(pan_dif)//speed_sides)*signal_pan
        rem_pan = int(abs(pan_dif)%speed_sides)*signal_pan

        tilt_dif = int(t) - read_tilt_open_loop_deg

        if tilt_dif < 0.0:
            signal_tilt = -1
            if not tracking_mode:
                u_tilt =  self.u_tilt_down
            else:
                u_tilt = 1
        else:
            signal_tilt = 1
            if not tracking_mode:
                u_tilt = self.u_tilt_up
            else:
                u_tilt = 1

        div_tilt = int(abs(tilt_dif)//u_tilt)*signal_tilt
        rem_tilt = int(abs(tilt_dif)%u_tilt)*signal_tilt

        
        print("PAN:", read_pan_open_loop_deg, " -> ", p, "dif:", pan_dif, "u_pan:", speed_sides, "div:", div_pan, "rem:", rem_pan)
        print("TILT:", read_tilt_open_loop_deg, " -> ", t, "dif:", tilt_dif, "u_tilt:", u_tilt, "div:", div_tilt, "rem:", rem_tilt)

        ctr = 1
        new_tilt = read_tilt_open_loop_deg+rem_tilt
        new_pan = read_pan_open_loop_deg+rem_pan

        if not tracking_mode:

            self.send_neck_move(new_pan, new_tilt)

            while(t != new_tilt or p!=new_pan):
                
                time.sleep(d_t)

                if new_pan == p:
                    new_pan = p
                else:
                    new_pan = read_pan_open_loop_deg+rem_pan+((ctr)*signal_pan*speed_sides)
                
                if new_tilt == t:
                    new_tilt = t
                else:
                    new_tilt = read_tilt_open_loop_deg+rem_tilt+((ctr)*signal_tilt*u_tilt)
                
                self.send_neck_move(new_pan, new_tilt)
                ctr+=1

        else:
            print("Set new variables for tracking!")
            self.tracking_target_p = p
            self.tracking_target_t = t
            self.tracking_ctr = 1
            self.tracking_new_pan = new_pan
            self.tracking_new_tilt = new_tilt
            self.tracking_read_pan_open_loop_deg = read_pan_open_loop_deg
            self.tracking_read_tilt_open_loop_deg = read_tilt_open_loop_deg
            self.tracking_signal_pan = signal_pan
            self.tracking_signal_tilt = signal_tilt
            self.tracking_u_pan = speed_sides
            self.tracking_u_tilt = u_tilt
            self.tracking_rem_pan = rem_pan
            self.tracking_rem_tilt = rem_tilt

        # print("ctr:", ctr)
        # print("END")


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
        
        # print(p,t)

        p = int(p * DEGREES_TO_SERVO_TICKS_CONST + 0.5)
        t = int(t * DEGREES_TO_SERVO_TICKS_CONST + 0.5)

        read_pan_open_loop = p
        read_tilt_open_loop = t  

        self.publish_get_neck_pos(read_pan_open_loop, read_tilt_open_loop)
        
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_PAN, ADDR_MX_GOAL_POSITION, p)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_TILT, ADDR_MX_GOAL_POSITION, t)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))


def main(args=None):
    rclpy.init(args=args)
    node = NeckNode()
    rclpy.spin(node)
    rclpy.shutdown()