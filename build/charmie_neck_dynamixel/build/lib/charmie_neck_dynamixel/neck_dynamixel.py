#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

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

# Control table address
ADDR_MX_TORQUE_ENABLE = 24  # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_PRESENT_POSITION = 36

ADDR_MX_D_GAIN = 26  # Control table address is different in Dynamixel model
ADDR_MX_I_GAIN = 27  # Control table address is different in Dynamixel model
ADDR_MX_P_GAIN = 28  # Control table address is different in Dynamixel model
D_GAIN = 0
I_GAIN = 0
P_GAIN = 10


# Protocol version
PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_TILT = 1  # Dynamixel ID : 1
DXL_ID_PAN = 2  # Dynamixel ID : 2
BAUDRATE = 57600  # Dynamixel default baudrate : 57600sssssss
# MAC GIL # DEVICENAME = '/dev/tty.usbserial-AI0282RX'  # Check which port is being used on your controller
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE = 700  # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE = 3200  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

pan = 2000
tilt = 2000

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]  # Goal position








class NeckNode(Node):

    def __init__(self):
        super().__init__("Neck")
        self.get_logger().info("Initialised CHARMIE Neck Node")
        # self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        # self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        # self.get_logger().info("Turtle controller node has been started")

    """
    def pose_callback(self, pose: Pose):
        cmd = Twist()
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publisher_.publish(cmd)
    """

def main(args=None):
    rclpy.init(args=args)

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

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

    orig_settings = termios.tcgetattr(sys.stdin)

    tty.setcbreak(sys.stdin)

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_D_GAIN, D_GAIN)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_I_GAIN, I_GAIN)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_P_GAIN, P_GAIN)

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_D_GAIN, D_GAIN)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_I_GAIN, I_GAIN)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_P_GAIN, P_GAIN)



    pan, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID_PAN, ADDR_MX_PRESENT_POSITION)
    #pan = dxl_present_position

    tilt, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID_TILT, ADDR_MX_PRESENT_POSITION)
    #tilt = dxl_present_position

    tilt_move = 0
    pan_move = 0


    node = NeckNode()
    rclpy.spin(node)
    rclpy.shutdown()
