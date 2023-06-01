#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Bool

import tty
import termios
import os

print(os.name)

if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


from dynamixel_sdk import *  # Uses Dynamixel SDK library

### PAN (ID = 1) down servo (left and right)
### TILT (ID = 2)  up servo (up and down)

# Control table address
ADDR_MX_TORQUE_ENABLE = 24  # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_PRESENT_POSITION = 36

ADDR_MX_D_GAIN = 26  # Control table address is different in Dynamixel model
ADDR_MX_I_GAIN = 27  # Control table address is different in Dynamixel model
ADDR_MX_P_GAIN = 28  # Control table address is different in Dynamixel model

# Different PID gains for each axis
PAN_D_GAIN = 0
PAN_I_GAIN = 1
PAN_P_GAIN = 2

# Different PID gains for each axis
TILT_D_GAIN = 2
TILT_I_GAIN = 4
TILT_P_GAIN = 3

# Protocol version
PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_PAN = 1  # Dynamixel ID : 1 
DXL_ID_TILT = 2  # Dynamixel ID : 2
BAUDRATE = 57600  # Dynamixel default baudrate : 57600sssssss
# MAC GIL # DEVICENAME = '/dev/tty.usbserial-AI0282RX'  # Check which port is being used on your controller
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE = 700  # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE = 3200  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

DEGREES_TO_SERVO_TICKS_CONST = 11.3777292
SERVO_TICKS_TO_DEGREES_CONST = 0.087891

MAX_PAN_ANGLE = 270
MIN_PAN_ANGLE = 90
MAX_TILT_ANGLE = 235
MIN_TILT_ANGLE = 120

pan = 2048
tilt = 2048

pan_corr = pan
tilt_corr = tilt

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]  # Goal position

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
        self.neck_position_subscriber = self.create_subscription(Pose2D, "neck_pos", self.neck_position_callback ,10)
        self.neck_error_subscriber = self.create_subscription(Pose2D, "neck_error", self.neck_error_callback , 10)
        self.neck_get_position_publisher = self.create_publisher(Pose2D, "get_neck_pos", 10)
        self.flag_neck_position_subscriber = self.create_subscription(Bool, "flag_neck_pos", self.flag_neck_position_callback , 10)

        self.create_timer(0.1, self.timer_callback)
        self.flag_get_neck_position = False
        self.k_e = 0.5


    def timer_callback(self):
        pan, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_PRESENT_POSITION)
        tilt, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_PRESENT_POSITION)
        
        global pan_corr, tilt_corr

        # when the servo is moving it usually returns values extremely high, this way these values are filtered
        if pan < 4096:
            pan_corr = pan
        if tilt < 4096:
            tilt_corr = tilt
        
        # print("PAN = ", pan, " TILT = ", tilt, " PAN_F = ", pan_corr, " TILT_F = ", tilt_corr)
        # print("PAN_F = ", int(pan_corr * SERVO_TICKS_TO_DEGREES_CONST + 0.5), " TILT_F = ", int(tilt_corr * SERVO_TICKS_TO_DEGREES_CONST + 0.5))

        # print( self.flag_get_neck_position)
        if  self.flag_get_neck_position:
            cmd = Pose2D()
            cmd.x = pan_corr * SERVO_TICKS_TO_DEGREES_CONST
            cmd.y = tilt_corr * SERVO_TICKS_TO_DEGREES_CONST
            self.neck_get_position_publisher.publish(cmd)


    def neck_position_callback(self, coords: Pose2D):
        self.get_logger().info("Received Neck Position, Adjusting the Neck Position")
        # print("Received Position: pan =", coords.x, " tilt = ", coords.y)
        move_neck(coords.x, coords.y)


    def neck_error_callback(self, error: Pose2D):
        self.get_logger().info("Received Neck Position (by Pixel Errors), Adjusting the Neck Position")
        move_neck((pan_corr * SERVO_TICKS_TO_DEGREES_CONST) - (error.x*self.k_e), (tilt_corr * SERVO_TICKS_TO_DEGREES_CONST) - (error.y*self.k_e))
        
    def flag_neck_position_callback(self, flag: Bool):
        # print("Flag Neck Position Set To: ", flag.data)
        if flag.data:
            self.get_logger().info("Received Reading Neck Position State True")
        else:
            self.get_logger().info("Received Reading Neck Position State False")
        self.flag_get_neck_position = flag.data
        

def move_neck(p, t):
    # print(p,t)

    if p > MAX_PAN_ANGLE:
        p = MAX_PAN_ANGLE
    if p < MIN_PAN_ANGLE:
        p = MIN_PAN_ANGLE

    if t > MAX_TILT_ANGLE:
        t = MAX_TILT_ANGLE
    if t < MIN_TILT_ANGLE:
        t = MIN_TILT_ANGLE

    p = int(p * DEGREES_TO_SERVO_TICKS_CONST + 0.5)
    t = int(t * DEGREES_TO_SERVO_TICKS_CONST + 0.5)
    
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
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_TORQUE_ENABLE,
                                                              TORQUE_ENABLE)
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
    node.get_logger().info("Set Torque Mode")

    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_D_GAIN, PAN_D_GAIN)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_I_GAIN, PAN_I_GAIN)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_P_GAIN, PAN_P_GAIN)

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_D_GAIN, TILT_D_GAIN)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_I_GAIN, TILT_I_GAIN)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_P_GAIN, TILT_P_GAIN)

    move_neck(180, 180) # resets the neck whenever the node is started, so that at the beginning the neck is always facing forward 
    node.get_logger().info("Set Neck to Initial Position, Looking Forward")

    rclpy.spin(node)
    rclpy.shutdown()
