#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from example_interfaces.msg import Bool

import threading
import traceback
import sys
import struct
import time
import serial
import math

# with the addition of the 2nd LIDAR, there may be a problem of port name.
# In low_level and neck i ahnged the port names to /dev/serial/by-id/
# However, both lidars seem to have the same id. So we have to change to port name or /dev/serial/by-path/ 
# These are the paths of connections to the robot:
# lrwxrwxrwx 1 root root 13 Feb 14 15:03 pci-0000:00:14.0-usb-0:4.2:1.0 -> ../../ttyACM0
# lrwxrwxrwx 1 root root 13 Feb 14 15:03 pci-0000:00:14.0-usb-0:4.3:1.0 -> ../../ttyACM1
# I will leave this here so when we actually implement the 2nd lidar we can change the port name
# Hoppefully it will be soon.

uart_port = '/dev/ttyACM0'
uart_speed = 19200

LASER_ANG_MAX = 119.885
LASER_STEP_DEG = 0.35208516886930985


def chunks(l, n):
    for i in range(0, len(l), n):
        yield l[i:i + n]


def decode(val):
    bin_str = '0b'
    for char in val:
        val = ord(char) - 0x30
        bin_str += '%06d' % int(bin(val)[2:])
    return int(bin_str, 2)


class Hokuyo(object):
    SHORT_COMMAND_LEN = 5
    MD_COMMAND_REPLY_LEN = 20

    LASER_ON = 'BM\n'
    LASER_OFF = 'QT\n'
    RESET = 'RS\n'

    VERSION_INFO = 'VV\n'
    SENSOR_STATE = 'II\n'
    SENSOR_SPECS = 'PP\n'
    SET_SCIP2    = 'SCIP2.0\n'

    CHARS_PER_VALUE = 3.0
    CHARS_PER_LINE = 66.0
    CHARS_PER_BLOCK = 64.0

    START_DEG = 119.885
    STEP_DEG = 0.35208516886930985

    START_STEP = 44
    STOP_STEP = 725

    VERSION_INFO_LINES = 6
    SENSOR_STATE_LINES = 8
    SENSOR_SPECS_LINES = 9

    def __init__(self, port):
        self.__port = port
        self.__port_lock = threading.RLock()

        self.__timestamp, self.__angles, self.__distances = 0, [], []
        self.__scan_lock = threading.Lock()

        self.__is_active = True
        self.__scanning_allowed = False

    def __offset(self):
        count = 2
        result = ''

        self.__port_lock.acquire()
        try:
            a = self.__port.read(1)
            b = self.__port.read(1)

            while not ((a == '\n' and b == '\n') or (a == '' and b == '')):
                result += a
                a = b
                b = self.__port.read(1)
                count += 1
        finally:
            self.__port_lock.release()

        result += a
        result += b

        sys.stderr.write('READ %d EXTRA BYTES: "%s"\n' % (count, str(result)))

    def __execute_command(self, command):
        self.__port_lock.acquire()
        try:
            self.__port.write(command)
            result = self.__port.read(len(command))
            assert result == command
        finally:
            self.__port_lock.release()
        return result

    def __short_command(self, command, check_response=True):
        result = ''
        self.__port_lock.acquire()
        try:
            try:
                result += self.__execute_command(command)
                result += self.__port.read(Hokuyo.SHORT_COMMAND_LEN)

                if check_response:
                    assert result[-5:-2] == '00P'
                assert result[-2:] == '\n\n'

                return result
            except BaseException as e:
                sys.stderr.write('RESULT: "%s"' % result)
                traceback.print_exc()
                self.__offset()
        finally:
            self.__port_lock.release()

    def __long_command(self, cmd, lines, check_response=True):
        result = ''
        self.__port_lock.acquire()
        try:
            try:
                result += self.__execute_command(cmd)

                result += self.__port.read(4)
                if check_response:
                    assert result[-4:-1] == '00P'
                assert result[-1:] == '\n'

                line = 0
                while line < lines:
                    char = self.__port.read_byte()
                    if not char is None:
                        char = chr(char)
                        result += char
                        if char == '\n':
                            line += 1
                    else:  # char is None
                        line += 1

                assert result[-2:] == '\n\n'

                return result
            except BaseException as e:
                sys.stderr.write('RESULT: "%s"' % result)
                traceback.print_exc()
                self.__offset()
        finally:
            self.__port_lock.release()

    def terminate(self):
        self.reset()

        self.__is_active = False
        self.__port_lock.acquire()
        try:
            self.__port.close()
        finally:
            self.__port_lock.release()

    def laser_on(self):
        return self.__short_command(Hokuyo.LASER_ON, check_response=True)


    def laser_off(self):
        return self.__short_command(Hokuyo.LASER_OFF)

    def reset(self):
        return self.__short_command(Hokuyo.RESET)

    def set_scip2(self):
        "for URG-04LX"
        return self.__short_command(Hokuyo.SET_SCIP2, check_response=False)

    def set_motor_speed(self, motor_speed=99):
        return self.__short_command('CR' + '%02d' % motor_speed + '\n', check_response=False)

    def set_high_sensitive(self, enable=True):
        return self.__short_command('HS' + ('1\n' if enable else '0\n'), check_response=False)

    def get_version_info(self):
        return self.__long_command(Hokuyo.VERSION_INFO, Hokuyo.VERSION_INFO_LINES)

    def get_sensor_state(self):
        return self.__long_command(Hokuyo.SENSOR_STATE, Hokuyo.SENSOR_STATE_LINES)

    def get_sensor_specs(self):
        return self.__long_command(Hokuyo.SENSOR_SPECS, Hokuyo.SENSOR_SPECS_LINES)

    def __get_and_parse_scan(self, cluster_count, start_step, stop_step):
        distances = {}
        result = ''

        count = ((stop_step - start_step) * Hokuyo.CHARS_PER_VALUE * Hokuyo.CHARS_PER_LINE)
        count /= (Hokuyo.CHARS_PER_BLOCK * cluster_count)
        count += 1.0 + 4.0  # paoolo(FIXME): why +4.0?
        count = int(count)

        self.__port_lock.acquire()
        try:
            result += self.__port.read(count)
        finally:
            self.__port_lock.release()

        assert result[-2:] == '\n\n'

        result = result.split('\n')
        result = [line[:-1] for line in result]
        result = ''.join(result)

        i = 0
        start = (-Hokuyo.START_DEG + Hokuyo.STEP_DEG * cluster_count * (start_step - Hokuyo.START_STEP))
        for chunk in chunks(result, 3):
            distances[- ((Hokuyo.STEP_DEG * cluster_count * i) + start)] = decode(chunk)
            i += 1

        return distances

    def get_single_scan(self, start_step=START_STEP, stop_step=STOP_STEP, cluster_count=1):
        self.__port_lock.acquire()
        try:
            cmd = 'GD%04d%04d%02d\n' % (start_step, stop_step, cluster_count)
            self.__port.write(cmd)

            result = self.__port.read(len(cmd))
            assert result == cmd

            result += self.__port.read(4)
            assert result[-4:-1] == '00P'
            assert result[-1] == '\n'

            result = self.__port.read(6)
            assert result[-1] == '\n'

            scan = self.__get_and_parse_scan(cluster_count, start_step, stop_step)
            return scan

        except BaseException as e:
            traceback.print_exc()
            self.__offset()

        finally:
            self.__port_lock.release()

    def __get_multiple_scans(self, start_step=START_STEP, stop_step=STOP_STEP, cluster_count=1,
                             scan_interval=0, number_of_scans=0):
        self.__port_lock.acquire()
        try:
            cmd = 'MD%04d%04d%02d%01d%02d\n' % (start_step, stop_step, cluster_count, scan_interval, number_of_scans)
            self.__port.write(cmd)

            result = self.__port.read(len(cmd))
            assert result == cmd

            result += self.__port.read(Hokuyo.SHORT_COMMAND_LEN)
            assert result[-2:] == '\n\n'

            index = 0
            while number_of_scans == 0 or index > 0:
                index -= 1

                result = self.__port.read(Hokuyo.MD_COMMAND_REPLY_LEN)
                assert result[:13] == cmd[:13]

                result = self.__port.read(6)
                assert result[-1] == '\n'

                scan = self.__get_and_parse_scan(cluster_count, start_step, stop_step)
                yield scan

        except BaseException as e:
            traceback.print_exc()
            self.__offset()

        finally:
            self.__port_lock.release()

    def enable_scanning(self, _enable_scanning):
        self.__scanning_allowed = _enable_scanning

    def __set_scan(self, scan):
        if scan is not None:
            timestamp = int(time.time() * 1000.0)
            angles, distances = Hokuyo.__parse_scan(scan)

            self.__scan_lock.acquire()
            try:
                self.__angles, self.__distances, self.__timestamp = angles, distances, timestamp
            finally:
                self.__scan_lock.release()

    def get_scan(self):
        if not self.__scanning_allowed:
            scan = self.get_single_scan()
            self.__set_scan(scan)

        self.__scan_lock.acquire()
        try:
            return self.__angles, self.__distances, self.__timestamp
        finally:
            self.__scan_lock.release()

    def scanning_loop(self):
        while self.__is_active:
            if self.__scanning_allowed:
                self.__port_lock.acquire()
                for scan in self.__get_multiple_scans():
                    self.__set_scan(scan)
                    if not self.__scanning_allowed or not self.__is_active:
                        self.laser_off()
                        self.laser_on()
                        self.__port_lock.release()
                        break
            time.sleep(0.1)

    @staticmethod
    def __parse_scan(scan):
        angles = sorted(scan.keys())
        distances = list(map(scan.get, angles))
        return angles, distances


class SerialPort(object):
    def __init__(self, serial_port):
        self.__port = serial_port
        self.__checksum = 0

    def close(self):
        self.__port.close()

    def get_checksum(self):
        return self.__checksum

    def read(self, size):
        char = self.__port.read(size)
        if sys.version_info >= (3, 0, 0):
            char = str(char, 'UTF-8')
        return char

    def write(self, char):
        if sys.version_info >= (3, 0, 0):
            char = bytes(char, 'UTF-8')
        self.__port.write(char)

    def send_command(self, address, command):
        self.__checksum = address
        self.__port.write(chr(address))
        self.__checksum += command
        self.__port.write(chr(command))

    def read_byte(self):
        res = self.__port.read(1)
        if len(res) > 0:
            val = struct.unpack('>B', res)
            self.__checksum += val[0] & 0xFF
            return val[0]
        return None

    def read_sbyte(self):
        res = self.__port.read(1)
        if len(res) > 0:
            val = struct.unpack('>b', res)
            self.__checksum += val[0] & 0xFF
            return val[0]
        return None

    def read_word(self):
        res = self.__port.read(2)
        if len(res) > 0:
            val = struct.unpack('>H', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            return val[0]
        return None

    def read_sword(self):
        res = self.__port.read(2)
        if len(res) > 0:
            val = struct.unpack('>h', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            return val[0]
        return None

    def read_long(self):
        res = self.__port.read(4)
        if len(res) > 0:
            val = struct.unpack('>L', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            self.__checksum += (val[0] >> 16) & 0xFF
            self.__checksum += (val[0] >> 24) & 0xFF
            return val[0]
        return None

    def read_slong(self):
        res = self.__port.read(4)
        if len(res) > 0:
            val = struct.unpack('>l', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            self.__checksum += (val[0] >> 16) & 0xFF
            self.__checksum += (val[0] >> 24) & 0xFF
            return val[0]
        return None

    def write_byte(self, val):
        self.__checksum += val & 0xFF
        return self.__port.write(struct.pack('>B', val))

    def write_sbyte(self, val):
        self.__checksum += val & 0xFF
        return self.__port.write(struct.pack('>b', val))

    def write_word(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        return self.__port.write(struct.pack('>H', val))

    def write_sword(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        return self.__port.write(struct.pack('>h', val))

    def write_long(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        self.__checksum += (val >> 16) & 0xFF
        self.__checksum += (val >> 24) & 0xFF
        return self.__port.write(struct.pack('>L', val))

    def write_slong(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        self.__checksum += (val >> 16) & 0xFF
        self.__checksum += (val >> 24) & 0xFF
        return self.__port.write(struct.pack('>l', val))


class LidarNode(Node):

    def __init__(self):
        super().__init__("Lidar")
        self.get_logger().info("Initialised CHARMIE LIDAR Node")
        
        self.lidar_publisher = self.create_publisher(LaserScan, "scan", 10)
        
        laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        port = SerialPort(laser_serial)

        self.laser = Hokuyo(port)
        self.laser.laser_on()
        self.get_logger().info("Started Hokuyo Scans")

        self.create_timer(0.1, self.timer_callback)
        

    def timer_callback(self):
        laser_scan = LaserScan()

        scan = self.laser.get_single_scan()
        ranges = []
        for key, value in scan.items():
            ranges.append(float(value/1000))
            # print(key, value)

        laser_scan.header.stamp = self.get_clock().now().to_msg()
        laser_scan.header.frame_id = 'lidar_frame'
        laser_scan.angle_min = -LASER_ANG_MAX*math.pi/180.0
        laser_scan.angle_max = LASER_ANG_MAX*math.pi/180.0
        laser_scan.angle_increment = LASER_STEP_DEG*math.pi/180.0
        laser_scan.time_increment = 0.0
        laser_scan.scan_time = 0.1
        laser_scan.range_min = 0.02
        laser_scan.range_max = 5.6
        laser_scan.ranges = ranges
        self.lidar_publisher.publish(laser_scan)
        # print(laser_scan)
        

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()
